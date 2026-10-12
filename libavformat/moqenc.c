/*
 * MoQ (Media over QUIC) muxer
 * Copyright (c) 2026 The FFmpeg Project
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Publishes video and/or audio streams to a MoQ relay via libmoq.
 *
 * The transport is owned by libmoq (AVFMT_NOFILE): the output URL is handed to
 * moq_endpoint_connect() verbatim, which understands moqt:// and https://.
 * Objects carry either LOC or CMAF packaging (-moq_packaging); CMAF chunks
 * come from a chained mp4 muxer.
 */
#include <errno.h>
#include <limits.h>
#include <string.h>

#include <moq/codec_signaling.h>
#include <moq/endpoint.h>
#include <moq/media_sender.h>
#include <moq/rcbuf.h>
#include <moq/wire.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"
#include "nal.h"
#include "url.h"

#define MOQ_NS_PARTS_MAX 32

#define MOQ_TIMESCALE 1000000

#define MOQ_CODEC_STR_MAX 64

#define MOQ_NAL_RECORD_VERSION 0x01

/* AV1CodecConfigurationRecord marker bit */
#define MOQ_AV1C_MARKER 0x80

/* The "40" in "mp4a.40.2". */
#define MOQ_AAC_OTI 0x40

#define MOQ_AUDIO_FRAGMENT_FRAMES_MAX 1024

#define MOQ_CMAF_MOVFLAGS                                                      \
    "+cmaf+frag_custom+empty_moov+default_base_moof+skip_trailer"

#define MOQ_CONNECT_TIMEOUT_US 5000000

/* Poll slice, so the interrupt callback is checked promptly. */
#define MOQ_READY_SLICE_US 100000

#define MOQ_DRAIN_TIMEOUT_US 5000000

enum {
    MOQ_PKG_LOC,
    MOQ_PKG_CMAF,
};

enum MOQLocInit {
    MOQ_LOC_INIT_DECODER_CONFIG,
    MOQ_LOC_INIT_EXTRADATA,
};

typedef moq_codec_source_format_t (*MOQSourceProbe)(const uint8_t *extradata,
                                                    int *is_annexb);

static moq_codec_source_format_t moq_probe_avc(const uint8_t *extradata,
                                               int *is_annexb)
{
    if (extradata[0] == MOQ_NAL_RECORD_VERSION)
        return MOQ_CODEC_SOURCE_AVC_AVCC;

    *is_annexb = 1;
    return MOQ_CODEC_SOURCE_AVC_ANNEXB;
}

static moq_codec_source_format_t moq_probe_hevc(const uint8_t *extradata,
                                                int *is_annexb)
{
    if (extradata[0] == MOQ_NAL_RECORD_VERSION)
        return MOQ_CODEC_SOURCE_HEVC_HVCC;

    *is_annexb = 1;
    return MOQ_CODEC_SOURCE_HEVC_ANNEXB;
}

static moq_codec_source_format_t moq_probe_av1(const uint8_t *extradata,
                                               int *is_annexb)
{
    return extradata[0] & MOQ_AV1C_MARKER ? MOQ_CODEC_SOURCE_AV1_AV1C
                                          : MOQ_CODEC_SOURCE_AV1_OBU;
}

typedef struct MOQCodecDesc {
    enum AVCodecID codec_id;

    /* Codecs with one extradata format set source_format; the rest probe. */
    moq_codec_source_format_t source_format;
    MOQSourceProbe probe;

    moq_codec_config_format_t config_format;
    const char *codec_prefix;

    uint8_t object_type_indication;

    uint32_t sample_entry;

    enum MOQLocInit loc_init;

    uint32_t decoded_sample_rate;
} MOQCodecDesc;

static const MOQCodecDesc moq_codecs[] = {
    {
        .codec_id      = AV_CODEC_ID_H264,
        .probe         = moq_probe_avc,
        .config_format = MOQ_CODEC_CONFIG_AVCC,
        .codec_prefix  = "avc1",
        .sample_entry  = MKTAG('a', 'v', 'c', '1'),
    },
    {
        .codec_id      = AV_CODEC_ID_HEVC,
        .probe         = moq_probe_hevc,
        .config_format = MOQ_CODEC_CONFIG_HVCC,
        .codec_prefix  = "hvc1",
        .sample_entry  = MKTAG('h', 'v', 'c', '1'),
    },
    {
        .codec_id      = AV_CODEC_ID_AV1,
        .probe         = moq_probe_av1,
        .config_format = MOQ_CODEC_CONFIG_AV1C,
        .codec_prefix  = "av01",
        .sample_entry  = MKTAG('a', 'v', '0', '1'),
    },
    {
        .codec_id      = AV_CODEC_ID_AAC,
        .source_format = MOQ_CODEC_SOURCE_AAC_ASC,
        .config_format = MOQ_CODEC_CONFIG_AAC_ASC,
        .codec_prefix  = "mp4a",
        .object_type_indication = MOQ_AAC_OTI,
        .sample_entry  = MKTAG('m', 'p', '4', 'a'),
    },
    {
        .codec_id      = AV_CODEC_ID_OPUS,
        .source_format = MOQ_CODEC_SOURCE_OPUS_HEAD,
        .config_format = MOQ_CODEC_CONFIG_OPUS,
        .codec_prefix  = "opus",
        .sample_entry  = MKTAG('O', 'p', 'u', 's'),
        .loc_init      = MOQ_LOC_INIT_EXTRADATA,
        .decoded_sample_rate = 48000,
    },
};

static const MOQCodecDesc *moq_find_codec(enum AVCodecID id)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(moq_codecs); i++)
        if (moq_codecs[i].codec_id == id)
            return &moq_codecs[i];

    return NULL;
}

typedef struct MOQTrack {
    AVStream *stream;
    moq_media_track_t *media;
    int is_audio;
    const MOQCodecDesc *desc;

    const char *name;

    char codec_str[MOQ_CODEC_STR_MAX];
    size_t codec_str_len;

    /* Annex-B input, which LOC has to reframe. */
    int is_annexb;

    /* CMAF only: chained mp4 muxer for this stream. */
    AVFormatContext *cmaf_mux;
    AVRational src_tb; /* before rescaling to microseconds */

    /* First dts; per track, as movenc rebases each track anyway. */
    int64_t ts_offset;
    AVRational frame_rate; /* 0/0 if unknown; video only */
    uint32_t cmaf_timescale; /* as declared by the init segment */
    uint64_t bitrate; /* max bitrate for the catalog; resolved at init */
    int no_duration_warned;


    int fragment_frames;
    int sent_frames;
} MOQTrack;

typedef struct MOQContext {
    const AVClass *av_class;

    char *namespace_str;
    char *video_track_name;
    char *audio_track_name;
    char *ca_file;
    int insecure;
    int packaging;
    int draft;
    int audio_fragment_frames;

    moq_version_t version_buf[2];

    char *ns_buf;
    moq_bytes_t ns_parts[MOQ_NS_PARTS_MAX];
    size_t ns_count;

    moq_endpoint_t *ep;
    moq_media_sender_t *tx;

    MOQTrack *tracks; /* one per AVStream, same index */
    int nb_tracks;
} MOQContext;

typedef struct MOQObject {
    const uint8_t *data;
    int size;
    int64_t pts, dts;
    int is_sync;
    int starts_group, ends_group;
    int64_t capture_time_us;
} MOQObject;

static av_always_inline int moq_is_cmaf(const MOQContext *ctx) {
    return ctx->packaging == MOQ_PKG_CMAF;
}

static int moq_err(AVFormatContext *s, moq_result_t res, const char *what)
{
    av_log(s, AV_LOG_ERROR, "%s: %s\n", what, moq_strerror(res));

    switch (res) {
    case MOQ_ERR_NOMEM:
        return AVERROR(ENOMEM);
    case MOQ_ERR_INVAL:
        return AVERROR(EINVAL);
    case MOQ_ERR_CLOSED:
        return AVERROR(EIO);
    case MOQ_ERR_INTERRUPTED:
        return AVERROR_EXIT;
    case MOQ_ERR_UNSUPPORTED:
        return AVERROR(ENOSYS);
    case MOQ_ERR_PROTO:
        return AVERROR_INVALIDDATA;
    default:
        return AVERROR_EXTERNAL;
    }
}

static int moq_split_namespace(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!ctx->namespace_str || !*ctx->namespace_str) {
        av_log(s, AV_LOG_ERROR, "The moq_namespace option is required, "
                                "e.g. -moq_namespace demo-live\n");
        return AVERROR(EINVAL);
    }

    ctx->ns_buf = av_strdup(ctx->namespace_str);
    if (!ctx->ns_buf)
        return AVERROR(ENOMEM);

    for (char *p = ctx->ns_buf;;) {
        char *hyphen = strchr(p, '-');

        if (hyphen)
            *hyphen = '\0';
        if (*p) {
            if (ctx->ns_count == MOQ_NS_PARTS_MAX) {
                av_log(s, AV_LOG_ERROR, "moq_namespace has more than %d "
                                        "components\n",
                       MOQ_NS_PARTS_MAX);
                return AVERROR(EINVAL);
            }
            ctx->ns_parts[ctx->ns_count].data = (const uint8_t *)p;
            ctx->ns_parts[ctx->ns_count].len  = strlen(p);
            ctx->ns_count++;
        }
        if (!hyphen)
            break;
        p = hyphen + 1;
    }

    if (!ctx->ns_count) {
        av_log(s, AV_LOG_ERROR, "moq_namespace '%s' has no components\n",
               ctx->namespace_str);
        return AVERROR(EINVAL);
    }

    return 0;
}

static void moq_close_fragmenter(MOQTrack *trk)
{
    if (!trk->cmaf_mux)
        return;

    ffio_free_dyn_buf(&trk->cmaf_mux->pb);
    avformat_free_context(trk->cmaf_mux);
    trk->cmaf_mux = NULL;
}

/* Open the CMAF fragmenter and capture its init segment. */
static int moq_open_fragmenter(AVFormatContext *s, MOQTrack *trk,
                               uint8_t **init, size_t *init_len) {
    AVStream *stream = trk->stream;
    AVDictionary *opts = NULL;
    AVFormatContext *c;
    AVStream *cst;
    uint8_t *buf;
    int len, ret;

    ret = avformat_alloc_output_context2(&trk->cmaf_mux, NULL, "mp4", NULL);
    if (ret < 0)
        return ret;
    c = trk->cmaf_mux;
    c->flags |= s->flags & AVFMT_FLAG_BITEXACT;

    cst = avformat_new_stream(c, NULL);
    if (!cst)
        return AVERROR(ENOMEM);
    if ((ret = avcodec_parameters_copy(cst->codecpar, stream->codecpar)) < 0)
        return ret;

    cst->codecpar->codec_tag = trk->desc->sample_entry;
    cst->time_base = trk->src_tb;

    if ((ret = avio_open_dyn_buf(&c->pb)) < 0)
        return ret;

    av_dict_set(&opts, "movflags", MOQ_CMAF_MOVFLAGS, 0);

    if (trk->frame_rate.num > 0)
        av_dict_set_int(&opts, "video_track_timescale", trk->frame_rate.num, 0);

    ret = avformat_write_header(c, &opts);
    av_dict_free(&opts);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Could not start the CMAF fragmenter\n");
        return ret;
    }

    trk->cmaf_timescale = cst->time_base.den;

    avio_flush(c->pb);
    len = avio_get_dyn_buf(c->pb, &buf);
    if (len <= 0) {
        av_log(s, AV_LOG_ERROR, "CMAF fragmenter produced no init segment\n");
        return AVERROR_EXTERNAL;
    }
    *init = av_memdup(buf, len);
    if (!*init)
        return AVERROR(ENOMEM);
    *init_len = len;
    ffio_reset_dyn_buf(c->pb);

    return 0;
}

 /* Hand one packet to the fragmenter and cut a chunk (one complete moof+mdat pair). */
static int moq_fragment(AVFormatContext *s, MOQTrack *trk, AVPacket *pkt,
                        uint8_t **out, int *out_len) {
    int64_t pts = pkt->pts;
    int64_t dts = pkt->dts;
    int64_t duration = pkt->duration;
    int ret;

    if (!pkt->duration) {
        if (trk->frame_rate.num > 0)
            pkt->duration = av_rescale_q(1, av_inv_q(trk->frame_rate),
                                         trk->stream->time_base);
        else if (!trk->no_duration_warned) {
            av_log(s, AV_LOG_WARNING, "Packets carry no duration and the stream "
                                      "declares no frame rate; CMAF chunks will "
                                      "have zero sample durations\n");
            trk->no_duration_warned = 1;
        }
    }

    if (trk->ts_offset == AV_NOPTS_VALUE) {
        trk->ts_offset = pkt->dts != AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        if (trk->ts_offset == AV_NOPTS_VALUE)
            trk->ts_offset = 0;
    }
    if (pkt->pts != AV_NOPTS_VALUE)
        pkt->pts -= trk->ts_offset;
    if (pkt->dts != AV_NOPTS_VALUE)
        pkt->dts -= trk->ts_offset;

    ret = ff_write_chained(trk->cmaf_mux, 0, pkt, s, 0);
    pkt->pts = pts;
    pkt->dts = dts;
    pkt->duration = duration;
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "CMAF fragmenter rejected a packet: %s\n",
               av_err2str(ret));
        return ret;
    }

    if ((ret = av_write_frame(trk->cmaf_mux, NULL)) < 0)
        return ret;

    avio_flush(trk->cmaf_mux->pb);
    *out_len = avio_get_dyn_buf(trk->cmaf_mux->pb, out);

    return 0;
}

static int moq_build_decoder_config(AVFormatContext *s, MOQTrack *trk,
                                    uint8_t **decoder_config,
                                    size_t *decoder_config_len) {
    const AVCodecParameters *codecpar = trk->stream->codecpar;
    const uint8_t *src = codecpar->extradata;
    int is_audio = trk->is_audio;
    uint8_t *buf = NULL;
    size_t len = 0;

    if (!src || codecpar->extradata_size <= 0) {
        av_log(s, AV_LOG_ERROR, "The %s stream %d carries no decoder "
               "configuration\n", is_audio ? "audio" : "video",
               trk->stream->index);
        return AVERROR(EINVAL);
    }

    moq_codec_init_data_cfg_t icfg;
    moq_codec_init_data_cfg_init(&icfg);
    icfg.source.data = src;
    icfg.source.len = codecpar->extradata_size;
    trk->is_annexb = 0;
    icfg.source_format = trk->desc->probe
                       ? trk->desc->probe(src, &trk->is_annexb)
                       : trk->desc->source_format;

    moq_result_t res = moq_codec_init_data_build(&icfg, NULL, 0, &len);
    if (res != MOQ_ERR_BUFFER)
        return moq_err(s, res, "Could not size the decoder configuration");

    buf = av_malloc(len);
    if (!buf)
        return AVERROR(ENOMEM);

    res = moq_codec_init_data_build(&icfg, buf, len, &len);
    if (res != MOQ_OK) {
        av_free(buf);
        return moq_err(s, res, "Could not build the decoder configuration");
    }

    *decoder_config = buf;
    *decoder_config_len = len;

    return 0;
}

static int moq_format_codec_string(AVFormatContext *s, MOQTrack *trk,
                                   const uint8_t *decoder_config,
                                   size_t decoder_config_len) {
    const MOQCodecDesc *desc = trk->desc;

    moq_codec_string_cfg_t scfg;
    moq_codec_string_cfg_init(&scfg);
    scfg.decoder_config.data = decoder_config;
    scfg.decoder_config.len = decoder_config_len;
    scfg.config_format = desc->config_format;
    scfg.sample_entry = moq_bytes_cstr(desc->codec_prefix);
    scfg.has_mp4_object_type_indication = desc->object_type_indication != 0;
    scfg.mp4_object_type_indication = desc->object_type_indication;

    moq_result_t res = moq_codec_string_format(&scfg, (uint8_t *)trk->codec_str,
                                               sizeof(trk->codec_str),
                                               &trk->codec_str_len);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not format the codec string");

    return 0;
}

static int moq_resolve_bitrate(AVFormatContext *s, MOQTrack *trk)
{
    const AVCodecParameters *codecpar = trk->stream->codecpar;
    const AVPacketSideData *sd;
    const AVCPBProperties *props;

    sd = av_packet_side_data_get(codecpar->coded_side_data,
                                 codecpar->nb_coded_side_data,
                                 AV_PKT_DATA_CPB_PROPERTIES);
    props = sd ? (const AVCPBProperties *)sd->data : NULL;
    if (props && props->max_bitrate > 0) {
        trk->bitrate = props->max_bitrate;
    } else if (codecpar->bit_rate > 0) {
        trk->bitrate = codecpar->bit_rate;
    } else {
        av_log(s, AV_LOG_ERROR, "The %s stream %d has no bitrate, which the "
               "catalog requires; set a maximum bitrate on the encoder "
               "(e.g. -maxrate:%c)\n", trk->is_audio ? "audio" : "video",
               trk->stream->index, trk->is_audio ? 'a' : 'v');
        return AVERROR(EINVAL);
    }

    return 0;
}

static int moq_group_tag(AVFormatContext *s, const MOQTrack *trk,
                         const char *key, bool *has, int *out)
{
    const AVDictionaryEntry *e = av_dict_get(trk->stream->metadata, key,
                                             NULL, 0);
    char *end;
    long v;

    if (!e)
        return 0;

    errno = 0;
    v = strtol(e->value, &end, 10);
    if (end == e->value || *end || errno == ERANGE || v < 0 || v > INT_MAX) {
        av_log(s, AV_LOG_ERROR, "Stream %d has %s='%s', which is not a "
               "non-negative integer\n", trk->stream->index, key, e->value);
        return AVERROR(EINVAL);
    }

    *out = v;
    *has = true;
    return 0;
}

static int moq_add_track(AVFormatContext *s, MOQTrack *trk,
                         const uint8_t *init_data, size_t init_data_len) {
    MOQContext *ctx = s->priv_data;
    const AVCodecParameters *codecpar = trk->stream->codecpar;
    int is_audio = trk->is_audio;
    const char *name = trk->name;
    int cmaf = moq_is_cmaf(ctx);
    char channel_config[16];
    const AVDictionaryEntry *lang = av_dict_get(trk->stream->metadata,
                                                "language", NULL, 0);

    moq_media_track_cfg_t tcfg;
    memset(&tcfg, 0, sizeof(tcfg));
    moq_media_track_cfg_init(&tcfg);
    tcfg.name = moq_bytes_cstr(name);
    tcfg.media_type = is_audio ? MOQ_MEDIA_TYPE_AUDIO : MOQ_MEDIA_TYPE_VIDEO;
    tcfg.codec.data = (const uint8_t *)trk->codec_str;
    tcfg.codec.len = trk->codec_str_len;
    tcfg.timescale = cmaf ? trk->cmaf_timescale : MOQ_TIMESCALE;
    tcfg.is_live = 1;
    tcfg.packaging = cmaf ? MOQ_MEDIA_PACKAGING_CMAF : MOQ_MEDIA_PACKAGING_RAW;
    tcfg.init_data.data = init_data;
    tcfg.init_data.len = init_data_len;

    if (is_audio) {
        snprintf(channel_config, sizeof(channel_config), "%d",
                 codecpar->ch_layout.nb_channels);
        tcfg.samplerate = trk->desc->decoded_sample_rate
                              ? trk->desc->decoded_sample_rate
                              : codecpar->sample_rate;
        tcfg.channel_config = moq_bytes_cstr(channel_config);
    } else {
        tcfg.width = codecpar->width;
        tcfg.height = codecpar->height;

        if (trk->frame_rate.num > 0)
            tcfg.framerate_millis =
                av_rescale(1000, trk->frame_rate.num, trk->frame_rate.den);
    }

    tcfg.bitrate = trk->bitrate;
    if (lang && *lang->value)
        tcfg.lang = moq_bytes_cstr(lang->value);

    int ret = moq_group_tag(s, trk, "moq_alt_group", &tcfg.has_alt_group,
                            &tcfg.alt_group);
    if (ret < 0)
        return ret;

    moq_result_t res = moq_media_sender_add_track(ctx->tx, &tcfg, &trk->media);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not add the track");

    av_log(s, AV_LOG_VERBOSE,
           "Published %s %s track '%s' as %.*s (%d bytes of init data",
           cmaf ? "CMAF" : "LOC", is_audio ? "audio" : "video", name,
           (int)trk->codec_str_len, trk->codec_str, (int)tcfg.init_data.len);
    if (tcfg.lang.len)
        av_log(s, AV_LOG_VERBOSE, ", lang %s", lang->value);
    if (tcfg.has_alt_group)
        av_log(s, AV_LOG_VERBOSE, ", altGroup %d", tcfg.alt_group);
    av_log(s, AV_LOG_VERBOSE, ")\n");

    return 0;
}

static int moq_create_track(AVFormatContext *s, MOQTrack *trk) {
    const AVCodecParameters *codecpar = trk->stream->codecpar;
    uint8_t *decoder_config = NULL, *init_segment = NULL;
    size_t decoder_config_len = 0, init_data_len = 0;
    const uint8_t *init_data;
    int ret;

    ret = moq_build_decoder_config(s, trk, &decoder_config,
                                   &decoder_config_len);
    if (ret < 0)
        return ret;

    ret = moq_format_codec_string(s, trk, decoder_config, decoder_config_len);
    if (ret < 0) {
        av_free(decoder_config);
        return ret;
    }

    if (moq_is_cmaf(s->priv_data)) {
        ret = moq_open_fragmenter(s, trk, &init_segment, &init_data_len);
        init_data = init_segment;
    } else if (trk->desc->loc_init == MOQ_LOC_INIT_EXTRADATA) {
        init_data     = codecpar->extradata;
        init_data_len = codecpar->extradata_size;
    } else {
        init_data     = decoder_config;
        init_data_len = decoder_config_len;
    }

    if (ret >= 0)
        ret = moq_add_track(s, trk, init_data, init_data_len);

    av_free(decoder_config);
    av_free(init_segment);

    if (ret < 0)
        moq_close_fragmenter(trk);

    return ret;
}

/* Auto: one second of audio per group, from the encoder's frame size. */
static void moq_resolve_audio_group(MOQTrack *trk, const AVPacket *pkt)
{
    const AVCodecParameters *codecpar = trk->stream->codecpar;
    int64_t frame_size = codecpar->frame_size;

    if (frame_size <= 0 && codecpar->sample_rate > 0)
        frame_size = av_rescale_q(pkt->duration, trk->stream->time_base,
                                  (AVRational){ 1, codecpar->sample_rate });

    trk->fragment_frames = 1;
    if (frame_size > 0 && codecpar->sample_rate > 0)
        trk->fragment_frames =
            av_clip64(av_rescale(codecpar->sample_rate, 1, frame_size),
                      1, MOQ_AUDIO_FRAGMENT_FRAMES_MAX);
}

static int moq_wait_ready(AVFormatContext *s)
{
    MOQContext *ctx  = s->priv_data;
    int64_t deadline = av_gettime_relative() + MOQ_CONNECT_TIMEOUT_US;

    while (!moq_media_sender_is_ready(ctx->tx)) {
        if (ff_check_interrupt(&s->interrupt_callback))
            return AVERROR_EXIT;
        if (moq_media_sender_is_fatal(ctx->tx)) {
            av_log(s, AV_LOG_ERROR, "Relay refused the session before it was "
                                    "ready (code 0x%" PRIx64 ")\n",
                   moq_media_sender_fatal_code(ctx->tx));
            return AVERROR(EIO);
        }
        if (moq_media_sender_is_closed(ctx->tx)) {
            av_log(s, AV_LOG_ERROR, "Relay closed the session before it was "
                                    "ready (code 0x%" PRIx64 ")\n",
                   moq_media_sender_fatal_code(ctx->tx));
            return AVERROR(EIO);
        }
        int64_t now = av_gettime_relative();
        if (now >= deadline) {
            av_log(s, AV_LOG_ERROR, "Timed out after %d ms waiting for the relay "
                                    "to accept namespace '%s'\n",
                   MOQ_CONNECT_TIMEOUT_US / 1000, ctx->namespace_str);
            return AVERROR(ETIMEDOUT);
        }

        moq_result_t res = moq_media_sender_wait(
            ctx->tx, FFMIN(deadline - now, MOQ_READY_SLICE_US));
        if (res == MOQ_ERR_INTERRUPTED)
            return AVERROR_EXIT;
        if (res < 0 && res != MOQ_ERR_CLOSED)
            return moq_err(s, res, "Failed while waiting for the relay");
    }

    return 0;
}

static int moq_capture_extradata(AVFormatContext *s, MOQTrack *trk,
                                 const AVPacket *pkt)
{
    AVCodecParameters *codecpar = trk->stream->codecpar;
    const AVPacketSideData *sd;
    int ret;

    if (codecpar->extradata_size > 0)
        return 0;

    sd = av_packet_side_data_get(pkt->side_data, pkt->side_data_elems,
                                 AV_PKT_DATA_NEW_EXTRADATA);
    if (!sd || !sd->size || sd->size > INT_MAX)
        return 0;

    if ((ret = ff_alloc_extradata(codecpar, sd->size)) < 0)
        return ret;
    memcpy(codecpar->extradata, sd->data, sd->size);

    av_log(s, AV_LOG_VERBOSE,
           "Recovered %d bytes of %s decoder configuration from the first "
           "packet of stream %d\n", codecpar->extradata_size,
           trk->is_audio ? "audio" : "video", trk->stream->index);

    return 0;
}

/* A lone stream of its type may fall back to the moq_*_track option; with
 * several, each must be named through its moq_track metadata. */
static int moq_resolve_track_name(AVFormatContext *s, MOQTrack *trk,
                                  int nb_of_type)
{
    MOQContext *ctx = s->priv_data;
    const AVDictionaryEntry *e = av_dict_get(trk->stream->metadata,
                                             "moq_track", NULL, 0);
    const char *name = e ? e->value : NULL;
    char type = trk->is_audio ? 'a' : 'v';

    if (!name && nb_of_type == 1)
        name = trk->is_audio ? ctx->audio_track_name : ctx->video_track_name;

    if (!name || !*name) {
        av_log(s, AV_LOG_ERROR, "Stream %d needs a catalog track name: with "
               "several %s streams, name each one with "
               "-metadata:s:%c:N moq_track=NAME\n", trk->stream->index,
               trk->is_audio ? "audio" : "video", type);
        return AVERROR(EINVAL);
    }

    trk->name = name;
    return 0;
}

static int moq_query_codec(enum AVCodecID id, int std_compliance)
{
    return moq_find_codec(id) != NULL;
}

static int moq_init(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!s->url || !*s->url) {
        av_log(s, AV_LOG_ERROR, "An output URL is required, "
                                "e.g. moqt://localhost:4433/moq-relay\n");
        return AVERROR(EINVAL);
    }

    if (s->nb_streams < 1) {
        av_log(s, AV_LOG_ERROR, "This muxer needs at least one video or "
                                "audio stream\n");
        return AVERROR(EINVAL);
    }

    int nb_video = 0, nb_audio = 0;

    for (int i = 0; i < s->nb_streams; i++) {
        enum AVMediaType type = s->streams[i]->codecpar->codec_type;

        if (type == AVMEDIA_TYPE_VIDEO) {
            nb_video++;
        } else if (type == AVMEDIA_TYPE_AUDIO) {
            nb_audio++;
        } else {
            const char *type_name = av_get_media_type_string(type);
            av_log(s, AV_LOG_ERROR, "Stream %d is %s; this muxer publishes "
                   "only video and audio streams\n", i,
                   type_name ? type_name : "unknown");
            return AVERROR(EINVAL);
        }
    }

    ctx->tracks = av_calloc(s->nb_streams, sizeof(*ctx->tracks));
    if (!ctx->tracks)
        return AVERROR(ENOMEM);
    ctx->nb_tracks = s->nb_streams;

    for (int i = 0; i < s->nb_streams; i++) {
        MOQTrack *trk = &ctx->tracks[i];
        const AVCodecParameters *codecpar = s->streams[i]->codecpar;

        trk->stream = s->streams[i];
        trk->is_audio = codecpar->codec_type == AVMEDIA_TYPE_AUDIO;

        int ret = moq_resolve_track_name(s, trk,
                                         trk->is_audio ? nb_audio : nb_video);
        if (ret < 0)
            return ret;

        trk->desc = moq_find_codec(codecpar->codec_id);
        if (!trk->desc) {
            av_log(s, AV_LOG_ERROR, "Unsupported codec %s\n",
                   avcodec_get_name(codecpar->codec_id));
            return AVERROR(EINVAL);
        }

        trk->src_tb = s->streams[i]->time_base;
        trk->ts_offset = AV_NOPTS_VALUE;
        trk->fragment_frames = ctx->audio_fragment_frames;

        ret = moq_resolve_bitrate(s, trk);
        if (ret < 0)
            return ret;

        if (!trk->is_audio) {
            AVRational fr = s->streams[i]->avg_frame_rate;
            if (fr.num <= 0 || fr.den <= 0)
                fr = s->streams[i]->r_frame_rate;
            if (fr.num > 0 && fr.den > 0)
                trk->frame_rate = fr;
        }

        avpriv_set_pts_info(s->streams[i], 64, 1, MOQ_TIMESCALE);
    }

    for (int i = 0; i < ctx->nb_tracks; i++)
        for (int j = 0; j < i; j++)
            if (!strcmp(ctx->tracks[i].name, ctx->tracks[j].name)) {
                av_log(s, AV_LOG_ERROR, "Streams %d and %d are both named "
                       "'%s'; catalog track names must be unique\n", j, i,
                       ctx->tracks[i].name);
                return AVERROR(EINVAL);
            }

    int ret = moq_split_namespace(s);
    if (ret < 0)
        return ret;

    moq_endpoint_cfg_t ecfg;
    moq_endpoint_cfg_init(&ecfg);
    ecfg.url                  = moq_bytes_cstr(s->url);
    ecfg.insecure_skip_verify = ctx->insecure;
    if (ctx->ca_file)
        ecfg.ca_file = moq_bytes_cstr(ctx->ca_file);

    if (ctx->draft) {
        ctx->version_buf[0]         = (moq_version_t)ctx->draft;
        ecfg.versions.policy        = MOQ_VERSION_POLICY_EXACT;
        ecfg.versions.version_count = 1;
    } else {
        ctx->version_buf[0]         = MOQ_VERSION_DRAFT_18;
        ctx->version_buf[1]         = MOQ_VERSION_DRAFT_16;
        ecfg.versions.policy        = MOQ_VERSION_POLICY_LIST;
        ecfg.versions.version_count = 2;
    }
    ecfg.versions.versions = ctx->version_buf;
    ecfg.versions.struct_size = sizeof(ecfg.versions);

    moq_result_t res = moq_endpoint_connect(&ecfg, &ctx->ep);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not connect to the relay");

    moq_media_sender_cfg_t scfg;
    moq_media_sender_cfg_init_live_sized(&scfg, sizeof(scfg));
    scfg.endpoint = NULL;
    scfg.namespace_.parts    = ctx->ns_parts;
    scfg.namespace_.count    = ctx->ns_count;
    scfg.publish_tracks      = 1;
    scfg.drop_without_demand = 1;

    res = moq_media_sender_attach(ctx->ep, &scfg, &ctx->tx);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not attach the media sender");

    for (int i = 0; i < ctx->nb_tracks; i++) {
        MOQTrack *trk = &ctx->tracks[i];

        if (trk->stream->codecpar->extradata_size <= 0) {
            av_log(s, AV_LOG_VERBOSE,
                   "Stream %d carries no decoder configuration yet; its track "
                   "is published once the first packet supplies one\n", i);
            continue;
        }

        if ((ret = moq_create_track(s, trk)) < 0)
            return ret;
    }

    for (int i = 0; i < ctx->nb_tracks; i++)
        if (ctx->tracks[i].media)
            return moq_wait_ready(s);

    return 0;
}

static uint64_t moq_timestamp(int64_t ts)
{
    if (ts == AV_NOPTS_VALUE || ts < 0)
        return 0;
    return FFMIN((uint64_t)ts, MOQ_QUIC_VARINT_MAX);
}

static int moq_send_object(AVFormatContext *s, MOQTrack *trk,
                           const MOQObject *moqObj) {
    MOQContext *ctx = s->priv_data;

    moq_rcbuf_t *payload = NULL;
    moq_result_t res = moq_rcbuf_create(moq_alloc_default(), moqObj->data,
                                        moqObj->size, &payload);
    /* Payload was copied, so the mp4 buffer can be reused. */
    if (trk->cmaf_mux)
        ffio_reset_dyn_buf(trk->cmaf_mux->pb);
    if (res != MOQ_OK)
        return AVERROR(ENOMEM);

    moq_media_send_object_t obj;
    memset(&obj, 0, sizeof(obj));
    obj.struct_size = sizeof(obj);
    obj.payload = payload;
    obj.properties = NULL;
    obj.is_sync = moqObj->is_sync;
    obj.starts_group = moqObj->starts_group;
    obj.ends_group = moqObj->ends_group;
    obj.presentation_time_us = moq_timestamp(moqObj->pts);
    obj.decode_time_us = moq_timestamp(moqObj->dts);
    obj.has_capture_time = 1;
    obj.capture_time_us = moq_timestamp(moqObj->capture_time_us);

    if (moq_is_cmaf(ctx) && moqObj->starts_group) {
        obj.has_sap_type = 1;
        obj.sap_type =
            trk->stream->codecpar->video_delay > 0 ? MOQ_SAP_TYPE_2 : MOQ_SAP_TYPE_1;
    }

    res = moq_media_sender_write(ctx->tx, trk->media, &obj);
    if (res == MOQ_OK)
        return 0;

    /* Ownership transfers only on MOQ_OK. */
    moq_rcbuf_decref(payload);

    switch (res) {
    case MOQ_ERR_WOULD_BLOCK:
        return 0;
    case MOQ_ERR_CLOSED:
        av_log(s, AV_LOG_ERROR, "Relay closed the session (code 0x%" PRIx64 ")\n",
               moq_media_sender_fatal_code(ctx->tx));
        return AVERROR(EIO);
    case MOQ_ERR_INTERRUPTED:
        return AVERROR_EXIT;
    default:
        return moq_err(s, res, "Could not publish a media object");
    }
}

static int moq_create_track_from_pkt(AVFormatContext *s, MOQTrack *trk, const AVPacket *pkt)
{
    int ret;

    if ((ret = moq_capture_extradata(s, trk, pkt)) < 0)
        return ret;

    if ((ret = moq_create_track(s, trk)) < 0)
        return ret;

    return moq_wait_ready(s);
}

static int moq_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    MOQContext *ctx = s->priv_data;
    MOQTrack *trk = &ctx->tracks[pkt->stream_index];
    const AVProducerReferenceTime *prft;
    size_t prft_size;
    uint8_t *nal_buf = NULL;
    uint8_t *data;
    int size;
    int key;
    int starts_group, ends_group = 0;
    int ret;

    if (!pkt->size)
        return 0;

    if (pkt->pts == AV_NOPTS_VALUE && pkt->dts == AV_NOPTS_VALUE) {
        av_log(s, AV_LOG_ERROR, "Packets carry no timestamps\n");
        return AVERROR(EINVAL);
    }
    // handle in-band init data
    if (!trk->media && (ret = moq_create_track_from_pkt(s, trk, pkt)) < 0)
        return ret;

    if (pkt->pts == AV_NOPTS_VALUE)
        pkt->pts = pkt->dts;

    key = !!(pkt->flags & AV_PKT_FLAG_KEY);

    if (moq_is_cmaf(ctx)) {
        ret = moq_fragment(s, trk, pkt, &data, &size);
        if (ret < 0)
            return ret;
        if (size <= 0)
            return 0;
    } else if (trk->is_annexb) {
        size = pkt->size;
        ret = ff_nal_parse_units_buf(pkt->data, &nal_buf, &size);
        if (ret < 0)
            return ret;
        if (size <= 0) {
            av_free(nal_buf);
            return 0;
        }
        data = nal_buf;
    } else {
        data = pkt->data;
        size = pkt->size;
    }

    prft = (const AVProducerReferenceTime *)av_packet_get_side_data(
        pkt, AV_PKT_DATA_PRFT, &prft_size);

    /* Video groups follow GOPs; audio groups span audio_fragment_frames. */
    starts_group = key;
    if (trk->is_audio) {
        if (!trk->fragment_frames)
            moq_resolve_audio_group(trk, pkt);

        starts_group = !trk->sent_frames;
        ends_group   = ++trk->sent_frames == trk->fragment_frames;
        if (ends_group)
            trk->sent_frames = 0;
    }

    MOQObject obj = {
        .data = data,
        .size = size,
        .pts = pkt->pts,
        .dts = pkt->dts,
        .is_sync = key,
        .starts_group = starts_group,
        .ends_group = ends_group,
        .capture_time_us = prft && prft_size == sizeof(*prft) &&
                                   prft->wallclock > 0
                               ? prft->wallclock
                               : av_gettime(),
    };

    ret = moq_send_object(s, trk, &obj);
    av_free(nal_buf);
    return ret;
}

static int moq_write_trailer(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!ctx->tx)
        return 0;

    for (int i = 0; i < ctx->nb_tracks; i++) {
        MOQTrack *trk = &ctx->tracks[i];

        if (!trk->media)
            av_log(s, AV_LOG_WARNING,
                   "Stream %d was never published: it produced no decoder "
                   "configuration\n", i);

        // safeguard against partial init data
        if (trk->cmaf_mux && trk->cmaf_mux->pb) {
            uint8_t *tail = NULL;
            int tail_len;

            av_write_trailer(trk->cmaf_mux);
            /* Should be empty: skip_trailer suppresses the mfra. */
            tail_len = avio_get_dyn_buf(trk->cmaf_mux->pb, &tail);
            if (tail_len > 0)
                av_log(s, AV_LOG_WARNING,
                       "Discarding %d bytes of trailing "
                       "CMAF data\n",
                       tail_len);

            ffio_reset_dyn_buf(trk->cmaf_mux->pb);
        }

        if (trk->media) {
            moq_result_t res = moq_media_sender_end_track(ctx->tx, trk->media);
            if (res != MOQ_OK && res != MOQ_ERR_CLOSED)
                av_log(s, AV_LOG_WARNING, "Could not end the track: %s\n",
                       moq_strerror(res));
        }
    }

    moq_result_t res = moq_media_sender_complete(ctx->tx);
    if (res != MOQ_OK && res != MOQ_ERR_CLOSED && res != MOQ_ERR_WRONG_STATE)
        av_log(s, AV_LOG_WARNING, "Could not complete the broadcast: %s\n",
               moq_strerror(res));

    if (ctx->ep) {
        res = moq_endpoint_drain(ctx->ep, MOQ_DRAIN_TIMEOUT_US);
        if (res == MOQ_DONE)
            av_log(s, AV_LOG_WARNING, "Timed out after %d s flushing queued "
                                      "data; the tail may be truncated\n",
                   MOQ_DRAIN_TIMEOUT_US / 1000000);
        else if (res != MOQ_OK && res != MOQ_ERR_CLOSED)
            av_log(s, AV_LOG_VERBOSE, "Could not flush queued data: %s\n",
                   moq_strerror(res));
    }

    return 0;
}

static void moq_deinit(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (ctx->tx) {
        moq_media_sender_destroy(ctx->tx);
        ctx->tx = NULL;
    }
    if (ctx->ep) {
        moq_endpoint_stop(ctx->ep);
        moq_endpoint_destroy(ctx->ep);
        ctx->ep = NULL;
    }
    for (int i = 0; i < ctx->nb_tracks; i++) {
        MOQTrack *trk = &ctx->tracks[i];

        trk->media = NULL;
        moq_close_fragmenter(trk);
    }
    av_freep(&ctx->tracks);
    ctx->nb_tracks = 0;

    av_freep(&ctx->ns_buf);
    ctx->ns_count = 0;
}

#define OFFSET(x) offsetof(MOQContext, x)
#define ENC       AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "moq_namespace", "Namespace to announce, hyphen-separated (e.g. demo-live)", OFFSET(namespace_str), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "moq_video_track", "Catalog name of the video track, when there is one video stream; several need -metadata:s:v:N moq_track=NAME", OFFSET(video_track_name), AV_OPT_TYPE_STRING, { .str = "video" }, 0, 0, ENC },
    { "moq_audio_track", "Catalog name of the audio track, when there is one audio stream; several need -metadata:s:a:N moq_track=NAME", OFFSET(audio_track_name), AV_OPT_TYPE_STRING, { .str = "audio" }, 0, 0, ENC },
    { "moq_ca_file", "CA bundle used to verify the relay certificate", OFFSET(ca_file), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "moq_insecure", "Skip relay certificate verification (testing only)", OFFSET(insecure), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { "moq_draft", "MoQT draft to negotiate", OFFSET(draft), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 18, ENC, .unit = "draft" },
        { "auto", "offer every supported draft, newest first, and let the relay choose", 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, ENC, .unit = "draft" },
        { "16", "draft-16 only; a relay without it fails to connect", 0, AV_OPT_TYPE_CONST, { .i64 = 16 }, 0, 0, ENC, .unit = "draft" },
        { "18", "draft-18 only; a relay without it fails to connect", 0, AV_OPT_TYPE_CONST, { .i64 = 18 }, 0, 0, ENC, .unit = "draft" },
    { "moq_packaging", "How media is packaged inside MoQ objects", OFFSET(packaging), AV_OPT_TYPE_INT, { .i64 = MOQ_PKG_LOC }, MOQ_PKG_LOC, MOQ_PKG_CMAF, ENC, .unit = "packaging" },
        { "loc", "Low Overhead Container: the access unit, LOC properties derived from timing", 0, AV_OPT_TYPE_CONST, { .i64 = MOQ_PKG_LOC }, 0, 0, ENC, .unit = "packaging" },
        { "cmaf", "CMAF chunks: one moof+mdat fragment per object", 0, AV_OPT_TYPE_CONST, { .i64 = MOQ_PKG_CMAF }, 0, 0, ENC, .unit = "packaging" },
    { "moq_audio_fragment_frames", "Audio frames per MoQ group, one frame per object; under CMAF the group is one CMAF fragment and each object one of its moof+mdat chunks", OFFSET(audio_fragment_frames), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, MOQ_AUDIO_FRAGMENT_FRAMES_MAX, ENC, .unit = "fragment_frames" },
        { "auto", "one second of audio per group", 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, ENC, .unit = "fragment_frames" },
    { NULL },
};

static const AVClass moq_muxer_class = {
    .class_name = "MoQ muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static int moq_check_bitstream(AVFormatContext *s, AVStream *st,
                               const AVPacket *pkt) {
    if (st->codecpar->codec_id == AV_CODEC_ID_AAC && pkt->size > 2 &&
        (AV_RB16(pkt->data) & 0xfff0) == 0xfff0)
      return ff_stream_add_bitstream_filter(st, "aac_adtstoasc", NULL);

    if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
        st->codecpar->extradata_size <= 0)
        return ff_stream_add_bitstream_filter(st, "extract_extradata", NULL);

    return 1;
}

const FFOutputFormat ff_moq_muxer = {
    .p.name = "moq",
    .p.long_name = NULL_IF_CONFIG_SMALL("MoQ (Media over QUIC)"),
    .p.video_codec = AV_CODEC_ID_H264,
    .p.audio_codec = AV_CODEC_ID_AAC,
    .p.flags = AVFMT_GLOBALHEADER | AVFMT_NOFILE | AVFMT_VARIABLE_FPS |
               AVFMT_TS_NONSTRICT,
    .p.priv_class = &moq_muxer_class,
    .priv_data_size = sizeof(MOQContext),
    .init = moq_init,
    .query_codec = moq_query_codec,
    .check_bitstream = moq_check_bitstream,
    .write_packet = moq_write_packet,
    .write_trailer = moq_write_trailer,
    .deinit = moq_deinit,
};
