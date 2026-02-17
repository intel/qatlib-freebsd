/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file cpa_sample_code_zlib.c
 *
 *
 * @ingroup sample_code
 *
 * @description
 *    This is sample code that uses zlib to compress/decompress data.
 *****************************************************************************/

#include "cpa_dc.h"
#include "cpa_sample_code_utils_common.h"
#include "cpa_sample_code_dc_utils.h"
#ifdef USER_SPACE
#include "zlib.h"
#else
#include <sys/param.h>
#if __FreeBSD_version >= 1300000
#include <contrib/zlib/zlib.h>
#else
#include <sys/zlib.h>
#endif
#define zlib_deflateInit2 deflateInit2
#define zlib_deflate deflate
#define zlib_deflateEnd deflateEnd
#define zlib_inflateInit2 inflateInit2
#define zlib_inflate inflate
#define zlib_inflateEnd inflateEnd
#define zlib_inflateReset inflateReset
#endif
#include "qat_compression_zlib.h"

#ifdef USER_SPACE
CpaStatus deflate_init(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    int ret = 0;
    stream->zalloc = (alloc_func)0;
    stream->zfree = (free_func)0;
    stream->opaque = (voidpf)0;
    stream->next_in = Z_NULL;
    stream->next_out = Z_NULL;
    stream->avail_in = stream->avail_out = stream->total_out = 0;
    stream->adler = 0;

    ret = deflateInit2(stream,
                       DEFLATE_DEF_LEVEL,
                       Z_DEFLATED,
                       -DEFLATE_DEF_WINBITS,
                       DEFLATE_DEF_MEMLEVEL,
                       Z_DEFAULT_STRATEGY);
    if (ret != Z_OK)
    {
        PRINT_ERR("Error in deflateInit2\n");
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}

/*Compress Date on a zlib stream*/
CpaStatus deflate_compress(struct z_stream_s *stream,
                           const Cpa8U *src,
                           Cpa32U slen,
                           Cpa8U *dst,
                           Cpa32U dlen,
                           int deflate_type)
{
#ifdef USE_ZLIB
    int ret = 0;

    stream->next_in = (Cpa8U *)src;
    stream->avail_in = slen;
    stream->next_out = (Cpa8U *)dst;
    stream->avail_out = dlen;
    ret = deflate(stream, deflate_type);
    if (ret != Z_STREAM_END && ret != Z_OK)
    {
        PRINT_ERR("Error in zlib_deflate, ret = %d\n", ret);
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}

CpaStatus inflate_init(z_stream *stream, CpaDcSessionState sessState)
{
#ifdef USE_ZLIB
    int ret = 0;
    stream->zalloc = (alloc_func)0;
    stream->zfree = (free_func)0;
    stream->opaque = (voidpf)0;
    stream->next_in = Z_NULL;
    stream->next_out = Z_NULL;
    stream->avail_in = stream->avail_out = stream->total_out = 0;
    stream->adler = 0;

    ret = inflateInit2(stream, -DEFLATE_DEF_WINBITS);
    if (ret != Z_OK)
    {
        PRINT_ERR("Error in inflateInit2, ret = %d\n", ret);
        return CPA_STATUS_FAIL;
    }
    if (sessState == CPA_DC_STATEFUL)
    {
#if ZLIB_VERNUM >= 0x1234
        ret = inflateReset2(stream, -DEFLATE_DEF_WINBITS);
#else
        ret = inflateReset(stream);
#endif
        if (ret != Z_OK)
        {
            PRINT_ERR("Error in inflateReset\n");
            return CPA_STATUS_FAIL;
        }
    }
#endif
    return CPA_STATUS_SUCCESS;
}

CpaStatus inflate_decompress(z_stream *stream,
                             const Cpa8U *src,
                             Cpa32U slen,
                             Cpa8U *dst,
                             Cpa32U dlen,
                             CpaDcSessionState sessState)
{
#ifdef USE_ZLIB
    int ret = 0;
    int flushFlag = Z_SYNC_FLUSH;

    if (sessState == CPA_DC_STATELESS)
    {
        flushFlag = Z_FULL_FLUSH;
    }

    stream->next_in = (Cpa8U *)src;
    stream->avail_in = slen;
    stream->next_out = (Cpa8U *)dst;
    stream->avail_out = dlen;

    ret = inflate(stream, flushFlag);
    if (ret != Z_OK && ret != Z_STREAM_END)
    {
        PRINT_ERR("Error in inflate, ret = %d\n", ret);
        PRINT_ERR("stream msg = %s\n", stream->msg);
        PRINT_ERR("stream adler = %u\n", (unsigned int)stream->adler);
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}

/*close zlib stream*/
void deflate_destroy(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    deflateEnd(stream);
#endif
}

/*close zlib stream*/
void inflate_destroy(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    inflateEnd(stream);
#endif
}
#endif
#ifdef KERNEL_SPACE
/* initialize a zlib stream*/
CpaStatus deflate_init(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    int ret = 0;

    ret = zlib_deflateInit2(stream,
                            DEFLATE_DEF_LEVEL,
                            Z_DEFLATED,
                            -DEFLATE_DEF_WINBITS,
                            DEFLATE_DEF_MEMLEVEL,
                            Z_DEFAULT_STRATEGY);
    if (ret != Z_OK)
    {
        PRINT_ERR("Error in zlib_deflateInit2\n");
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(deflate_init);

/*Compress Date on a zlib stream*/
CpaStatus deflate_compress(struct z_stream_s *stream,
                           const Cpa8U *src,
                           Cpa32U slen,
                           Cpa8U *dst,
                           Cpa32U dlen,
                           int inflate_type)
{
#ifdef USE_ZLIB
    int ret = 0;

    stream->next_in = (Cpa8U *)src;
    stream->avail_in = slen;
    stream->next_out = (Cpa8U *)dst;
    stream->avail_out = dlen;

    ret = zlib_deflate(stream, inflate_type);
    if (ret != Z_STREAM_END && ret != Z_OK)
    {
        PRINT_ERR("Error in zlib_deflate, ret = %d\n", ret);
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(deflate_compress);

/*close zlib stream*/
void deflate_destroy(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    zlib_deflateEnd(stream);
#endif
}
EXPORT_SYMBOL(deflate_destroy);

CpaStatus inflate_init(z_stream *stream, CpaDcSessionState sessState)
{
#ifdef USE_ZLIB
    int ret = 0;

    ret = zlib_inflateInit2(stream, -MAX_WBITS);
    if (ret != Z_OK)
    {
        PRINT_ERR("Error in zlib_inflateInit2, ret = %d\n", ret);
        return CPA_STATUS_FAIL;
    }
    if (sessState == CPA_DC_STATEFUL)
    {
        ret = zlib_inflateReset(stream);
        if (ret != Z_OK)
        {
            PRINT_ERR("Error in zlib_inflateReset, ret = %d\n", ret);
            return CPA_STATUS_FAIL;
        }
    }
#endif
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(inflate_init);

CpaStatus inflate_decompress(z_stream *stream,
                             const Cpa8U *src,
                             Cpa32U slen,
                             Cpa8U *dst,
                             Cpa32U dlen,
                             CpaDcSessionState sessState)
{
#ifdef USE_ZLIB
    int ret = 0;
    int flushFlag = Z_SYNC_FLUSH;

    if (sessState == CPA_DC_STATELESS)
    {
        flushFlag = Z_FULL_FLUSH;
    }

    stream->next_in = (Cpa8U *)src;
    stream->avail_in = slen;
    stream->next_out = (Cpa8U *)dst;
    stream->avail_out = dlen;

    ret = zlib_inflate(stream, flushFlag);
    if (ret != Z_OK && ret != Z_STREAM_END)
    {
        PRINT_ERR("Error in zlib_inflate, ret = %d\n", ret);
        PRINT_ERR("stream msg = %s\n", stream->msg);
        PRINT_ERR("stream adler = %u\n", (unsigned int)stream->adler);
        return CPA_STATUS_FAIL;
    }
#endif
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(inflate_decompress);

/*close zlib stream*/
void inflate_destroy(struct z_stream_s *stream)
{
#ifdef USE_ZLIB
    zlib_inflateEnd(stream);
#endif
}
EXPORT_SYMBOL(inflate_destroy);
#endif
