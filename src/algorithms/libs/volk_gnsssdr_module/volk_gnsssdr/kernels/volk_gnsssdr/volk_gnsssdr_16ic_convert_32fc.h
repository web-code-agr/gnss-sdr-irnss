/*!
 * \file volk_gnsssdr_16ic_convert_32fc.h
 * \brief VOLK_GNSSSDR kernel: converts 16 bit integer complex complex values to 32 bits float complex values.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

/*!
 * \page volk_gnsssdr_16ic_convert_32fc
 *
 * \b Overview
 *
 * Converts a complex vector of 16-bits integer each component
 * into a complex vector of 32-bits float each component.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_convert_32fc(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
 * \endcode
 *
 * \b Inputs
 * \li inputVector:  The complex 16-bit integer input data buffer.
 * \li num_points:   The number of data values to be converted.
 *
 * \b Outputs
 * \li outputVector: pointer to a vector holding the converted vector.
 *
 */


#ifndef INCLUDED_volk_gnsssdr_16ic_convert_32fc_H
#define INCLUDED_volk_gnsssdr_16ic_convert_32fc_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_convert_32fc_generic(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            outputVector[i] = lv_cmake((float)lv_creal(inputVector[i]), (float)lv_cimag(inputVector[i]));
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_a_sse2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 2;
    unsigned int i;
    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m128 a;

    for (i = 0; i < sse_iters; i++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            _mm_store_ps((float*)_out, a);
            _in += 2;
            _out += 2;
        }
    for (i = 0; i < (num_points % 2); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in), (float)lv_cimag(*_in));
            _in++;
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_u_sse2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 2;
    unsigned int i;
    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m128 a;

    for (i = 0; i < sse_iters; i++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            _mm_storeu_ps((float*)_out, a);
            _in += 2;
            _out += 2;
        }
    for (i = 0; i < (num_points % 2); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in), (float)lv_cimag(*_in));
            _in++;
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_u_axv(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int i;
    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m256 a;

    for (i = 0; i < sse_iters; i++)
        {
            a = _mm256_set_ps((float)(lv_cimag(_in[3])), (float)(lv_creal(_in[3])), (float)(lv_cimag(_in[2])), (float)(lv_creal(_in[2])), (float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            _mm256_storeu_ps((float*)_out, a);
            _in += 4;
            _out += 4;
        }

    for (i = 0; i < (num_points % 4); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in), (float)lv_cimag(*_in));
            _in++;
        }
}
#endif /* LV_HAVE_AVX */

#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_a_axv(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int i;
    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m256 a;

    for (i = 0; i < sse_iters; i++)
        {
            a = _mm256_set_ps((float)(lv_cimag(_in[3])), (float)(lv_creal(_in[3])), (float)(lv_cimag(_in[2])), (float)(lv_creal(_in[2])), (float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            _mm256_store_ps((float*)_out, a);
            _in += 4;
            _out += 4;
        }

    for (i = 0; i < (num_points % 4); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in), (float)lv_cimag(*_in));
            _in++;
        }
}
#endif /* LV_HAVE_AVX */

#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_a_avx2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;
    const int16_t* complexVectorPtr = (int16_t*)inputVector;
    float* outputVectorPtr = (float*)outputVector;
    __m256 outVal;
    __m256i outValInt;
    __m128i cplxValue;

    for (number = 0; number < avx_iters; number++)
        {
            cplxValue = _mm_load_si128((__m128i*)complexVectorPtr);
            complexVectorPtr += 8;

            outValInt = _mm256_cvtepi16_epi32(cplxValue);
            outVal = _mm256_cvtepi32_ps(outValInt);
            _mm256_store_ps((float*)outputVectorPtr, outVal);

            outputVectorPtr += 8;
        }

    number = avx_iters * 8;
    for (; number < num_points * 2; number++)
        {
            *outputVectorPtr++ = (float)*complexVectorPtr++;
        }
}

#endif /* LV_HAVE_AVX2 */

#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_u_avx2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;
    const int16_t* complexVectorPtr = (int16_t*)inputVector;
    float* outputVectorPtr = (float*)outputVector;
    __m256 outVal;
    __m256i outValInt;
    __m128i cplxValue;

    for (number = 0; number < avx_iters; number++)
        {
            cplxValue = _mm_loadu_si128((__m128i*)complexVectorPtr);
            complexVectorPtr += 8;

            outValInt = _mm256_cvtepi16_epi32(cplxValue);
            outVal = _mm256_cvtepi32_ps(outValInt);
            _mm256_storeu_ps((float*)outputVectorPtr, outVal);

            outputVectorPtr += 8;
        }

    number = avx_iters * 8;
    for (; number < num_points * 2; number++)
        {
            *outputVectorPtr++ = (float)*complexVectorPtr++;
        }
}

#endif /* LV_HAVE_AVX2 */

#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_convert_32fc_neon(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 2;
    unsigned int i;
    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;

    int16x4_t a16x4;
    int32x4_t a32x4;
    float32x4_t f32x4;

    for (i = 0; i < sse_iters; i++)
        {
            a16x4 = vld1_s16((const int16_t*)_in);
            __VOLK_GNSSSDR_PREFETCH(_in + 4);
            a32x4 = vmovl_s16(a16x4);
            f32x4 = vcvtq_f32_s32(a32x4);
            vst1q_f32((float32_t*)_out, f32x4);
            _in += 2;
            _out += 2;
        }
    for (i = 0; i < (num_points % 2); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in), (float)lv_cimag(*_in));
            _in++;
        }
}
#endif /* LV_HAVE_NEON */

#endif /* INCLUDED_volk_gnsssdr_32fc_convert_16ic_H */
