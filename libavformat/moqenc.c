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
 * Publishes one encoded H.264 video stream to a MoQ relay via libmoq.
 *
 * The transport is owned by libmoq (AVFMT_NOFILE): the output URL is handed to
 * moq_endpoint_connect() verbatim, which understands moqt:// and https://.
 * Objects are published with RAW packaging, so the service derives the LOC-01
 * property block from the typed timing fields and the MSF catalog from the
 * track configuration; this muxer never hand-builds either.
 */

#include <string.h>

#include <moq/codec_signaling.h>
#include <moq/endpoint.h>
#include <moq/media_sender.h>
#include <moq/rcbuf.h>
#include <moq/wire.h>

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "avformat.h"
#include "internal.h"
#include "mux.h"
#include "nal.h"
#include "url.h"

/* A MoQ namespace carries at most 32 parts. */
#define MOQ_NS_PARTS_MAX 32

/* Microseconds, so the stream time base matches the track timescale. */
// TODO Validate same as OBS
#define MOQ_TIMESCALE 1000000

/* moq_codec_string_format() formats into a 64-byte scratch internally, so a
 * buffer this size never needs the two-call sizing dance. */
#define MOQ_CODEC_STR_MAX 64

/* MSF-01 5.2.22 requires a non-zero bitrate on a video track, but codecpar
 * carries none for a remux of a stream whose container recorded none. */
#define MOQ_DEFAULT_VIDEO_BITRATE 2000000

/* First byte of an avcC record, distinguishing it from the elementary-stream
 * form: configurationVersion is 1, and an Annex-B access unit cannot open with
 * that because a start code begins 0x00. */
#define MOQ_AVCC_FIRST_BYTE 0x01

/* How long init() waits for the relay to accept the namespace. */
#define MOQ_CONNECT_TIMEOUT_US 5000000

/* Readiness polling granularity, so the interrupt callback is honoured promptly
 * within that wait. */
#define MOQ_READY_SLICE_US 100000

/* How long the trailer waits for the local stream queues to flush. Without this
 * the tail of a finite input is still queued when the endpoint stops, and the
 * recording is truncated. */
#define MOQ_DRAIN_TIMEOUT_US 5000000

typedef struct MOQContext {
    const AVClass *av_class;

    /* options */
    char *namespace_str;
    char *video_track_name;
    char *ca_file;
    int insecure;

    /* namespace storage; ns_parts point into ns_buf */
    char *ns_buf;
    moq_bytes_t ns_parts[MOQ_NS_PARTS_MAX];
    size_t ns_count;

    /* libmoq state */
    moq_endpoint_t *ep;
    moq_media_sender_t *tx;
    moq_media_track_t *track;

    /* avcC built from extradata: the codec string is derived from it and it is
     * the catalog init_data. */
    uint8_t *config;
    size_t config_len;

    char codec_str[MOQ_CODEC_STR_MAX];
    size_t codec_str_len;

    /* Set when the packets carry Annex-B NAL units, which LOC has to reframe.
     * See the note in moq_write_packet(). */
    int annexb;
} MOQContext;

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

static int moq_create_track(AVFormatContext *s)
{
    MOQContext *ctx              = s->priv_data;
    const AVCodecParameters *par = s->streams[0]->codecpar;
    const uint8_t *src           = par->extradata;

    if (!src || par->extradata_size <= 0) {
        av_log(s, AV_LOG_ERROR, "The stream carries no decoder configuration\n");
        return AVERROR(EINVAL);
    }

    moq_codec_init_data_cfg_t icfg;
    moq_codec_init_data_cfg_init(&icfg);
    icfg.source_format = src[0] == MOQ_AVCC_FIRST_BYTE ? MOQ_CODEC_SOURCE_AVC_AVCC
                                                       : MOQ_CODEC_SOURCE_AVC_ANNEXB;
    icfg.source.data   = src;
    icfg.source.len    = par->extradata_size;

    ctx->annexb = icfg.source_format == MOQ_CODEC_SOURCE_AVC_ANNEXB;

    size_t need      = 0;
    moq_result_t res = moq_codec_init_data_build(&icfg, NULL, 0, &need);
    if (res != MOQ_ERR_BUFFER)
        return moq_err(s, res, "Could not size the decoder configuration");

    ctx->config = av_malloc(need);
    if (!ctx->config)
        return AVERROR(ENOMEM);

    res = moq_codec_init_data_build(&icfg, ctx->config, need, &need);
    if (res != MOQ_OK) {
        av_freep(&ctx->config);
        return moq_err(s, res, "Could not build the decoder configuration");
    }
    ctx->config_len = need;

    moq_codec_string_cfg_t scfg;
    moq_codec_string_cfg_init(&scfg);
    scfg.config_format       = MOQ_CODEC_CONFIG_AVCC;
    scfg.sample_entry        = moq_bytes_cstr("avc1");
    scfg.decoder_config.data = ctx->config;
    scfg.decoder_config.len  = ctx->config_len;

    res = moq_codec_string_format(&scfg, (uint8_t *)ctx->codec_str,
                                  sizeof(ctx->codec_str), &ctx->codec_str_len);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not format the codec string");

    moq_media_track_cfg_t tcfg;
    moq_media_track_cfg_init(&tcfg);
    tcfg.name           = moq_bytes_cstr(ctx->video_track_name);
    tcfg.media_type     = MOQ_MEDIA_TYPE_VIDEO;
    tcfg.codec.data     = (const uint8_t *)ctx->codec_str;
    tcfg.codec.len      = ctx->codec_str_len;
    tcfg.timescale      = MOQ_TIMESCALE;
    tcfg.is_live        = 1;
    tcfg.packaging      = MOQ_MEDIA_PACKAGING_RAW;
    tcfg.init_data.data = ctx->config;
    tcfg.init_data.len  = ctx->config_len;
    tcfg.width          = par->width;
    tcfg.height         = par->height;

    if (par->bit_rate > 0) {
        tcfg.bitrate = par->bit_rate;
    } else {
        tcfg.bitrate = MOQ_DEFAULT_VIDEO_BITRATE;
    }

    res = moq_media_sender_add_track(ctx->tx, &tcfg, &ctx->track);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not add the track");

    av_log(s, AV_LOG_VERBOSE, "Published track '%s' as %.*s (%d bytes of init "
                              "data)\n",
           ctx->video_track_name, (int)ctx->codec_str_len, ctx->codec_str,
           (int)tcfg.init_data.len);

    return 0;
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

static int moq_init(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!s->url || !*s->url) {
        av_log(s, AV_LOG_ERROR, "An output URL is required, "
                                "e.g. moqt://localhost:4433/moq-relay\n");
        return AVERROR(EINVAL);
    }

    if (s->nb_streams != 1 ||
        s->streams[0]->codecpar->codec_id != AV_CODEC_ID_H264) {
        av_log(s, AV_LOG_ERROR, "This muxer publishes exactly one h264 video "
                                "stream\n");
        return AVERROR(EINVAL);
    }

    avpriv_set_pts_info(s->streams[0], 64, 1, MOQ_TIMESCALE);

    int ret = moq_split_namespace(s);
    if (ret < 0)
        return ret;

    static const moq_version_t versions[] = { MOQ_VERSION_DRAFT_16 };
    moq_endpoint_cfg_t ecfg;
    moq_endpoint_cfg_init(&ecfg);
    ecfg.url                  = moq_bytes_cstr(s->url);
    ecfg.insecure_skip_verify = ctx->insecure;
    if (ctx->ca_file)
        ecfg.ca_file = moq_bytes_cstr(ctx->ca_file);
    ecfg.versions.policy        = MOQ_VERSION_POLICY_EXACT;
    ecfg.versions.versions      = versions;
    ecfg.versions.version_count = FF_ARRAY_ELEMS(versions);

    moq_result_t res = moq_endpoint_connect(&ecfg, &ctx->ep);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not connect to the relay");

    moq_media_sender_cfg_t scfg;
    moq_media_sender_cfg_init_live(&scfg);
    scfg.endpoint            = NULL;   /* use our endpoint to allow  */
    scfg.namespace_.parts    = ctx->ns_parts;
    scfg.namespace_.count    = ctx->ns_count;
    scfg.publish_tracks      = 1;
    scfg.drop_without_demand = 1;

    res = moq_media_sender_attach(ctx->ep, &scfg, &ctx->tx);
    if (res != MOQ_OK)
        return moq_err(s, res, "Could not attach the media sender");

    if ((ret = moq_create_track(s)) < 0)
        return ret;

    return moq_wait_ready(s);
}

static uint64_t moq_timestamp(int64_t ts)
{
    if (ts == AV_NOPTS_VALUE || ts < 0)
        return 0;
    return FFMIN((uint64_t)ts, MOQ_QUIC_VARINT_MAX);
}

static int moq_send_packet(AVFormatContext *s, const AVPacket *pkt,
                           moq_media_track_t *track, int is_sync,
                           int starts_group, int ends_group)
{
    MOQContext *ctx  = s->priv_data;
    uint8_t *nal_buf = NULL;
    uint8_t *data    = pkt->data;
    int size         = pkt->size;

    if (ctx->annexb) {
        int ret = ff_nal_parse_units_buf(pkt->data, &nal_buf, &size);
        if (ret < 0)
            return ret;
        data = nal_buf;
    }

    moq_rcbuf_t *payload = NULL;
    moq_result_t res     = moq_rcbuf_create(moq_alloc_default(), data, size, &payload);
    av_free(nal_buf);
    if (res != MOQ_OK)
        return AVERROR(ENOMEM);

    moq_media_send_object_t obj;
    memset(&obj, 0, sizeof(obj));
    obj.struct_size          = sizeof(obj);
    obj.payload              = payload;
    obj.properties           = NULL;
    obj.is_sync              = is_sync;
    obj.starts_group         = starts_group;
    obj.ends_group           = ends_group;
    obj.presentation_time_us = moq_timestamp(pkt->pts);
    obj.decode_time_us       = moq_timestamp(pkt->dts);

    res = moq_media_sender_write(ctx->tx, track, &obj);
    if (res == MOQ_OK)
        return 0;

    /* Ownership only transferred on MOQ_OK. */
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

static int moq_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    MOQContext *ctx = s->priv_data;

    if (!pkt->size)
        return 0;

    int key = !!(pkt->flags & AV_PKT_FLAG_KEY);
    /* One group per GOP: a keyframe opens a group, which implicitly closes the
     * previous one, so no object needs ends_group. The last group is closed by
     * the end_track() in moq_write_trailer(). */
    return moq_send_packet(s, pkt, ctx->track, key, key, 0);
}

static int moq_write_trailer(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!ctx->tx)
        return 0;

    if (ctx->track) {
        moq_result_t res = moq_media_sender_end_track(ctx->tx, ctx->track);
        if (res != MOQ_OK && res != MOQ_ERR_CLOSED)
            av_log(s, AV_LOG_WARNING, "Could not end the track: %s\n",
                   moq_strerror(res));
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
    ctx->track = NULL;

    av_freep(&ctx->config);
    av_freep(&ctx->ns_buf);
    ctx->ns_count = 0;
}

#define OFFSET(x) offsetof(MOQContext, x)
#define ENC       AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "moq_namespace", "Namespace to announce, hyphen-separated (e.g. demo-live)", OFFSET(namespace_str), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "moq_video_track", "Catalog name of the published video track", OFFSET(video_track_name), AV_OPT_TYPE_STRING, { .str = "video" }, 0, 0, ENC },
    { "moq_ca_file", "CA bundle used to verify the relay certificate", OFFSET(ca_file), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "moq_insecure", "Skip relay certificate verification (testing only)", OFFSET(insecure), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { NULL },
};

static const AVClass moq_muxer_class = {
    .class_name = "MoQ muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_moq_muxer = {
    .p.name         = "moq",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MoQ (Media over QUIC)"),
    .p.video_codec  = AV_CODEC_ID_H264,
    .p.flags        = AVFMT_GLOBALHEADER | AVFMT_NOFILE |
                      AVFMT_VARIABLE_FPS | AVFMT_TS_NONSTRICT,
    .p.priv_class   = &moq_muxer_class,
    .priv_data_size = sizeof(MOQContext),
    .init           = moq_init,
    .write_packet   = moq_write_packet,
    .write_trailer  = moq_write_trailer,
    .deinit         = moq_deinit,
};
