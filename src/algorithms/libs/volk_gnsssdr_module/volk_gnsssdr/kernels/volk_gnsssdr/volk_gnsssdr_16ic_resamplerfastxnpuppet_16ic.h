/*!
 * \file volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic.h
 * \brief VOLK_GNSSSDR puppet for the multiple 16-bit complex vector resampler kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the multiple resampler into the test system
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

#ifndef INCLUDED_volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float code_phase_step_chips = 0.1;
    int code_length_chips = 2046;
    int num_out_vectors = 3;
    int n;
    float* rem_code_phase_chips = (float*)volk_gnsssdr_malloc(sizeof(float) * num_out_vectors, volk_gnsssdr_get_alignment());
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            rem_code_phase_chips[n] = -0.234;
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }
    volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn_generic(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);
    volk_gnsssdr_free(rem_code_phase_chips);
    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_a_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float code_phase_step_chips = 0.1;
    int code_length_chips = 2046;
    int num_out_vectors = 3;
    int n;
    float* rem_code_phase_chips = (float*)volk_gnsssdr_malloc(sizeof(float) * num_out_vectors, volk_gnsssdr_get_alignment());
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            rem_code_phase_chips[n] = -0.234;
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }
    volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn_a_sse2(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_out_vectors, num_points);

    memcpy(result, result_aux[0], sizeof(lv_16sc_t) * num_points);
    volk_gnsssdr_free(rem_code_phase_chips);
    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_u_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float code_phase_step_chips = 0.1;
    int code_length_chips = 2046;
    int num_out_vectors = 3;
    int n;
    float* rem_code_phase_chips = (float*)volk_gnsssdr_malloc(sizeof(float) * num_out_vectors, volk_gnsssdr_get_alignment());
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            rem_code_phase_chips[n] = -0.234;
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }
    volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn_u_sse2(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_out_vectors, num_points);

    memcpy(result, result_aux[0], sizeof(lv_16sc_t) * num_points);
    volk_gnsssdr_free(rem_code_phase_chips);
    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float code_phase_step_chips = 0.1;
    int code_length_chips = 2046;
    int num_out_vectors = 3;
    int n;
    float* rem_code_phase_chips = (float*)volk_gnsssdr_malloc(sizeof(float) * num_out_vectors, volk_gnsssdr_get_alignment());
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            rem_code_phase_chips[n] = -0.234;
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }
    volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn_neon(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_out_vectors, num_points);

    memcpy(result, result_aux[0], sizeof(lv_16sc_t) * num_points);
    volk_gnsssdr_free(rem_code_phase_chips);
    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#endif  // INCLUDED_volk_gnsssdr_16ic_resamplerpuppet_16ic_H
