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
 * Objects carry either RAW or CMAF packaging (-moq_packaging); CMAF chunks
 * come from a chained mp4 muxer.
 */

#include <string.h>

#include <moq/codec_signaling.h>
#include <moq/endpoint.h>
#include <moq/media_sender.h>
#include <moq/rcbuf.h>
#include <moq/wire.h>

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

/* Microseconds, so the stream time base matches the track timescale. */
#define MOQ_TIMESCALE 1000000

#define MOQ_CODEC_STR_MAX 64

/* MSF-01 5.2.22 requires a non-zero bitrate on a video track, but codecpar
 * carries none for a remux of a stream whose container recorded none. */
#define MOQ_DEFAULT_VIDEO_BITRATE 2000000

#define MOQ_AVCC_FIRST_BYTE 0x01

/* flags to control that one cmaf chunk becomes one MoQ Object and that the
 * init segment is emitted by itself */
#define MOQ_CMAF_MOVFLAGS                                                      \
  "+cmaf+frag_custom+empty_moov+default_base_moof+skip_trailer"

#define MOQ_CONNECT_TIMEOUT_US 5000000

/* Readiness polling granularity, so the interrupt callback is honoured promptly
 * within that wait. */
#define MOQ_READY_SLICE_US 100000

/* How long the trailer waits for the local stream queues to flush. */
#define MOQ_DRAIN_TIMEOUT_US 5000000

enum {
  MOQ_PKG_LOC,
  MOQ_PKG_CMAF,
};

typedef struct MOQContext {
    const AVClass *av_class;

    /* options */
    char *namespace_str;
    char *video_track_name;
    char *ca_file;
    int insecure;
    int packaging;
    int draft;

    moq_version_t version_buf[2];

    /* namespace storage; ns_parts point into ns_buf */
    char *ns_buf;
    moq_bytes_t ns_parts[MOQ_NS_PARTS_MAX];
    size_t ns_count;

    /* libmoq state */
    moq_endpoint_t *ep;
    moq_media_sender_t *tx;
    moq_media_track_t *track;

    char codec_str[MOQ_CODEC_STR_MAX];
    size_t codec_str_len;

    int annexb;

    /* MOQ_PKG_CMAF only: a chained mp4 muxer carrying our one stream. The
     * init segment it produced does not live here; add_track copies it. */
    AVFormatContext *mp4;
    int64_t ts_offset; /* first dts, subtracted from the child */
    AVRational src_tb; /* stream timebase before it is set to microseconds */
    AVRational frame_rate; /* 0/0 when the stream declares none */
    uint32_t cmaf_timescale; /* what the init segment ended up declaring */
    int no_duration_warned;
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

/**
 * Open the chained mp4 muxer that turns our packets into CMAF chunks, and
 * capture the init segment it emits.
 */
static int moq_open_fragmenter(AVFormatContext *s, uint8_t **init,
                               size_t *init_len) {
  MOQContext *ctx = s->priv_data;
  AVStream *st = s->streams[0];
  AVDictionary *opts = NULL;
  AVFormatContext *c;
  AVStream *cst;
  uint8_t *buf;
  int len, ret;

  ret = avformat_alloc_output_context2(&ctx->mp4, NULL, "mp4", NULL);
  if (ret < 0)
    return ret;
  c = ctx->mp4;
  c->flags |= s->flags & AVFMT_FLAG_BITEXACT;

  cst = avformat_new_stream(c, NULL);
  if (!cst)
    return AVERROR(ENOMEM);
  if ((ret = avcodec_parameters_copy(cst->codecpar, st->codecpar)) < 0)
    return ret;
  cst->time_base = ctx->src_tb;

  if ((ret = avio_open_dyn_buf(&c->pb)) < 0)
    return ret;

  av_dict_set(&opts, "movflags", MOQ_CMAF_MOVFLAGS, 0);
 
  if (ctx->frame_rate.num > 0)
    av_dict_set_int(&opts, "video_track_timescale", ctx->frame_rate.num, 0);
  
  ret = avformat_write_header(c, &opts);
  av_dict_free(&opts);
  if (ret < 0) {
    av_log(s, AV_LOG_ERROR, "Could not start the CMAF fragmenter\n");
    return ret;
  }

  ctx->cmaf_timescale = cst->time_base.den;

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

/**
 * Hand one packet to the fragmenter and cut a chunk (one complete moof+mdat pair).
 */
static int moq_fragment(AVFormatContext *s, AVPacket *pkt, uint8_t **out,
                        int *out_len) {
  MOQContext *ctx = s->priv_data;
  int64_t pts = pkt->pts;
  int64_t dts = pkt->dts;
  int64_t duration = pkt->duration;
  int ret;

  /* One sample per fragment leaves movenc no next dts to diff agains */
  if (!pkt->duration) {
    if (ctx->frame_rate.num > 0)
      pkt->duration = av_rescale_q(1, av_inv_q(ctx->frame_rate),
                                   s->streams[0]->time_base);
    else if (!ctx->no_duration_warned) {
      av_log(s, AV_LOG_WARNING, "Packets carry no duration and the stream "
                                "declares no frame rate; CMAF chunks will "
                                "have zero sample durations\n");
      ctx->no_duration_warned = 1;
    }
  }

  if (ctx->ts_offset == AV_NOPTS_VALUE) {
    ctx->ts_offset = pkt->dts != AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
    if (ctx->ts_offset == AV_NOPTS_VALUE)
      ctx->ts_offset = 0;
  }
  if (pkt->pts != AV_NOPTS_VALUE)
    pkt->pts -= ctx->ts_offset;
  if (pkt->dts != AV_NOPTS_VALUE)
    pkt->dts -= ctx->ts_offset;

  /* ff_write_chained() restores pkt's stream_index afterwards; it does not
   * copy the packet, so the rebase above and the restore below bracket it. */
  ret = ff_write_chained(ctx->mp4, 0, pkt, s, 0);
  pkt->pts = pts;
  pkt->dts = dts;
  pkt->duration = duration;
  if (ret < 0) {
    av_log(s, AV_LOG_ERROR, "CMAF fragmenter rejected a packet: %s\n",
           av_err2str(ret));
    return ret;
  }

  if ((ret = av_write_frame(ctx->mp4, NULL)) < 0)
    return ret;

  avio_flush(ctx->mp4->pb);
  *out_len = avio_get_dyn_buf(ctx->mp4->pb, out);

  return 0;
}

static int moq_build_codec_config(AVFormatContext *s, uint8_t **init_data,
                                  size_t *init_data_len) {
  MOQContext *ctx = s->priv_data;
  const AVCodecParameters *par = s->streams[0]->codecpar;
  const uint8_t *src = par->extradata;
  uint8_t *buf = NULL;
  size_t len = 0;

  if (!src || par->extradata_size <= 0) {
    av_log(s, AV_LOG_ERROR, "The stream carries no decoder configuration\n");
    return AVERROR(EINVAL);
  }

  moq_codec_init_data_cfg_t icfg;
  moq_codec_init_data_cfg_init(&icfg);
  icfg.source_format = src[0] == MOQ_AVCC_FIRST_BYTE
                           ? MOQ_CODEC_SOURCE_AVC_AVCC
                           : MOQ_CODEC_SOURCE_AVC_ANNEXB;
  icfg.source.data = src;
  icfg.source.len = par->extradata_size;

  ctx->annexb = icfg.source_format == MOQ_CODEC_SOURCE_AVC_ANNEXB;

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

  moq_codec_string_cfg_t scfg;
  moq_codec_string_cfg_init(&scfg);
  scfg.config_format = MOQ_CODEC_CONFIG_AVCC;
  scfg.sample_entry = moq_bytes_cstr("avc1");
  scfg.decoder_config.data = buf;
  scfg.decoder_config.len = len;

  res = moq_codec_string_format(&scfg, (uint8_t *)ctx->codec_str,
                                sizeof(ctx->codec_str), &ctx->codec_str_len);
  if (res != MOQ_OK) {
    av_free(buf);
    return moq_err(s, res, "Could not format the codec string");
  }

  *init_data = buf;
  *init_data_len = len;

  return 0;
}

static int moq_add_video_track(AVFormatContext *s, const uint8_t *init_data,
                               size_t init_data_len) {
  MOQContext *ctx = s->priv_data;
  const AVCodecParameters *par = s->streams[0]->codecpar;
  int cmaf = moq_is_cmaf(ctx);

  moq_media_track_cfg_t tcfg;
  memset(&tcfg, 0, sizeof(tcfg));
  moq_media_track_cfg_init(&tcfg);
  tcfg.name = moq_bytes_cstr(ctx->video_track_name);
  tcfg.media_type = MOQ_MEDIA_TYPE_VIDEO;
  tcfg.codec.data = (const uint8_t *)ctx->codec_str;
  tcfg.codec.len = ctx->codec_str_len;
  tcfg.timescale = cmaf ? ctx->cmaf_timescale : MOQ_TIMESCALE;
  tcfg.is_live = 1;
  tcfg.width = par->width;
  tcfg.height = par->height;
  tcfg.packaging = cmaf ? MOQ_MEDIA_PACKAGING_CMAF : MOQ_MEDIA_PACKAGING_RAW;
  tcfg.init_data.data = init_data;
  tcfg.init_data.len = init_data_len;

  if (ctx->frame_rate.num > 0)
    tcfg.framerate_millis =
        av_rescale(1000, ctx->frame_rate.num, ctx->frame_rate.den);

  if (par->bit_rate > 0) {
    tcfg.bitrate = par->bit_rate;
  } else {
    tcfg.bitrate = MOQ_DEFAULT_VIDEO_BITRATE;
  }

  moq_result_t res = moq_media_sender_add_track(ctx->tx, &tcfg, &ctx->track);
  if (res != MOQ_OK)
    return moq_err(s, res, "Could not add the track");

  av_log(s, AV_LOG_VERBOSE,
         "Published %s track '%s' as %.*s (%d bytes of init data)\n",
         cmaf ? "CMAF" : "LOC", ctx->video_track_name, (int)ctx->codec_str_len,
         ctx->codec_str, (int)tcfg.init_data.len);

  return 0;
}

static int moq_create_track(AVFormatContext *s) {
  uint8_t *init_data = NULL;
  size_t init_data_len = 0;
  int ret;

  ret = moq_build_codec_config(s, &init_data, &init_data_len);
  if (ret < 0)
    return ret;

  if (moq_is_cmaf(s->priv_data)) {
    av_freep(&init_data);
    init_data_len = 0;
    ret = moq_open_fragmenter(s, &init_data, &init_data_len);
  }
  
  if (ret >= 0)
    ret = moq_add_video_track(s, init_data, init_data_len);

  av_free(init_data);
  return ret;
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

    ctx->ts_offset = AV_NOPTS_VALUE;

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

    ctx->src_tb = s->streams[0]->time_base;
    ctx->frame_rate = s->streams[0]->avg_frame_rate;
    if (ctx->frame_rate.num <= 0 || ctx->frame_rate.den <= 0)
        ctx->frame_rate = s->streams[0]->r_frame_rate;
    if (ctx->frame_rate.num <= 0 || ctx->frame_rate.den <= 0)
        ctx->frame_rate = (AVRational){ 0, 0 };

    avpriv_set_pts_info(s->streams[0], 64, 1, MOQ_TIMESCALE);

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

static int moq_send_object(AVFormatContext *s, moq_media_track_t *track,
                           const MOQObject *moqObj) {
  MOQContext *ctx = s->priv_data;

  moq_rcbuf_t *payload = NULL;
  moq_result_t res = moq_rcbuf_create(moq_alloc_default(), moqObj->data,
                                      moqObj->size, &payload);
  /* CMAF data was copied into rcbuf, so the mp4 buffer is free to reuse. */
  if (ctx->mp4)
    ffio_reset_dyn_buf(ctx->mp4->pb);
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
    obj.sap_type = s->streams[0]->codecpar->video_delay > 0 ? MOQ_SAP_TYPE_2
                                                            : MOQ_SAP_TYPE_1;
  }

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
  const AVProducerReferenceTime *prft;
  size_t prft_size;
  uint8_t *nal_buf = NULL;
  uint8_t *data;
  int size;
  int key;
  int ret;

  if (!pkt->size)
    return 0;

  if (pkt->pts == AV_NOPTS_VALUE && pkt->dts == AV_NOPTS_VALUE) {
    av_log(s, AV_LOG_ERROR, "Packets carry no timestamps\n");
    return AVERROR(EINVAL);
  }
  if (pkt->pts == AV_NOPTS_VALUE)
    pkt->pts = pkt->dts;

  key = !!(pkt->flags & AV_PKT_FLAG_KEY);

  if (moq_is_cmaf(ctx)) {
    ret = moq_fragment(s, pkt, &data, &size);
    if (ret < 0)
      return ret;
    if (size <= 0)
      return 0;
  } else if (ctx->annexb) {
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

  MOQObject obj = {
      .data = data,
      .size = size,
      .pts = pkt->pts,
      .dts = pkt->dts,
      .is_sync = key,
      .starts_group = key,
      .capture_time_us = prft && prft_size == sizeof(*prft) &&
                                 prft->wallclock > 0
                             ? prft->wallclock
                             : av_gettime(),
  };

  ret = moq_send_object(s, ctx->track, &obj);
  av_free(nal_buf);
  return ret;
}

static int moq_write_trailer(AVFormatContext *s)
{
    MOQContext *ctx = s->priv_data;

    if (!ctx->tx)
        return 0;

    // safeguard against partial init data
    if (ctx->mp4 && ctx->mp4->pb) {
      uint8_t *tail = NULL;
      int tail_len;

      av_write_trailer(ctx->mp4);
      /* Nothing should be pending: one fragment is flushed per packet and
        skip_trailer suppresses the mfra. */
      tail_len = avio_get_dyn_buf(ctx->mp4->pb, &tail);
      if (tail_len > 0)
        av_log(s, AV_LOG_WARNING, "Discarding %d bytes of trailing CMAF data\n",
               tail_len);

      ffio_reset_dyn_buf(ctx->mp4->pb);
    }

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

    if (ctx->mp4) {
      /* avformat_free_context does not touch pb, so the dynamic buffer has
       * to go first. */
      ffio_free_dyn_buf(&ctx->mp4->pb);
      avformat_free_context(ctx->mp4);
      ctx->mp4 = NULL;
    }

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
    { "moq_draft", "MoQT draft to negotiate", OFFSET(draft), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 18, ENC, .unit = "draft" },
        { "auto", "offer every supported draft, newest first, and let the relay choose", 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, ENC, .unit = "draft" },
        { "16", "draft-16 only; a relay without it fails to connect", 0, AV_OPT_TYPE_CONST, { .i64 = 16 }, 0, 0, ENC, .unit = "draft" },
        { "18", "draft-18 only; a relay without it fails to connect", 0, AV_OPT_TYPE_CONST, { .i64 = 18 }, 0, 0, ENC, .unit = "draft" },
    { "moq_packaging", "How media is packaged inside MoQ objects", OFFSET(packaging), AV_OPT_TYPE_INT, { .i64 = MOQ_PKG_LOC }, MOQ_PKG_LOC, MOQ_PKG_CMAF, ENC, .unit = "packaging" },
        { "loc", "Low Overhead Container: the access unit, LOC properties derived from timing", 0, AV_OPT_TYPE_CONST, { .i64 = MOQ_PKG_LOC }, 0, 0, ENC, .unit = "packaging" },
        { "cmaf", "CMAF chunks: one moof+mdat fragment per object", 0, AV_OPT_TYPE_CONST, { .i64 = MOQ_PKG_CMAF }, 0, 0, ENC, .unit = "packaging" },
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
