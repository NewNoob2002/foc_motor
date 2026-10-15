/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_vectors.h"

#include <math.h> /* sinf, cosf */

#include "foc/foc_pi.h"
#include "foc/foc_svpwm.h"

#define FOC_VECTORS_INV_SQRT3 0.57735027f

/*
 * Columns: ia, ic, theta, id_ref, iq_ref, vbus, int_d, int_q.
 * Rows 0-7 are chosen cases; rows 8-63 are uniform random draws (fixed seed)
 * over ia, ic, iq_ref in [-10, 10] A, id_ref in [-5, 5] A, theta in [0, 2 pi),
 * vbus in [10, 24] V and the integrals in [-5, 5] V.
 */
const foc_vector_in_t foc_vectors_in[FOC_VECTORS_COUNT] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 24.0f, 0.0f, 0.0f},        /* all zero: duties 0.5 */
    {1.0f, -0.5f, 0.0f, 1.0f, 0.0f, 24.0f, 0.0f, 0.0f},       /* id = 1 A on the d axis, no error */
    {0.0f, -0.866f, 1.5708f, 0.0f, 0.0f, 24.0f, 1.0f, -1.0f}, /* theta = pi/2: current on beta = the d axis */
    {0.0f, 0.0f, 0.5f, 0.0f, 10.0f, 24.0f, 0.0f, 0.0f},       /* q PI saturates on kp alone */
    {2.0f, 3.0f, 2.0f, 5.0f, -10.0f, 10.0f, 4.0f, -4.0f},     /* low vbus: both axes saturate, SVPWM limits */
    {0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 24.0f, 20.0f, -20.0f},     /* integrals beyond the limit get clamped */
    {-3.0f, 1.5f, 6.2831f, -1.0f, 2.0f, 12.0f, 0.0f, 0.5f},   /* theta just below 2 pi */
    {5.0f, -5.0f, 3.1416f, 0.0f, 0.0f, 24.0f, 0.0f, 0.0f},    /* theta = pi, b = 0 */
    {8.287f, -6.031f, 3.022f, 3.2f, -9.194f, 18.54f, 0.252f, -0.775f},
    {-2.085f, 3.052f, 6.1594f, 1.312f, 3.062f, 23.93f, 3.306f, 2.897f},
    {-0.047f, 7.05f, 0.3649f, -2.658f, -6.401f, 23.87f, -2.288f, -1.487f},
    {8.603f, -4.599f, 2.4557f, 3.477f, -9.091f, 21.41f, 4.246f, 3.015f},
    {0.753f, -1.496f, 5.5678f, 1.46f, 3.204f, 16.95f, -3.857f, -4.307f},
    {-6.189f, -9.257f, 2.7705f, 3.802f, -5.922f, 10.26f, 4.448f, 0.614f},
    {1.644f, 4.236f, 6.1411f, 4.942f, -2.405f, 22.99f, 2.159f, -3.668f},
    {-3.897f, -9.342f, 3.2303f, -1.476f, -5.205f, 18.57f, -0.454f, 2.248f},
    {-4.095f, -9.216f, 4.522f, -1.984f, -5.117f, 18.78f, -0.571f, 3.423f},
    {-4.663f, 5.164f, 4.6975f, 3.704f, 1.514f, 22.58f, 4.137f, 4.813f},
    {-9.737f, 5.49f, 3.7898f, -3.755f, -2.105f, 16.65f, -3.887f, -3.086f},
    {-1.971f, -2.933f, 2.1665f, -4.524f, 8.79f, 11.21f, 4.803f, -4.064f},
    {6.12f, 8.628f, 1.143f, 0.001f, -4.056f, 14.57f, 3.756f, -3.517f},
    {8.363f, 3.423f, 3.709f, 0.628f, -4.912f, 14.37f, -1.065f, 3.194f},
    {1.341f, 6.656f, 0.3169f, 1.331f, 9.111f, 15.17f, 3.125f, 3.328f},
    {-4.068f, 6.828f, 5.6855f, -3.568f, 2.251f, 20.87f, -1.204f, -3.314f},
    {-7.106f, 8.625f, 5.6781f, -4.641f, 5.698f, 22.82f, 2.06f, -1.432f},
    {-8.854f, 1.368f, 1.0993f, 1.695f, -2.669f, 15.35f, -0.813f, 1.743f},
    {3.734f, -7.613f, 5.6702f, 0.296f, 1.995f, 18.72f, -0.986f, -4.435f},
    {-5.686f, -7.94f, 0.3776f, -3.767f, -8.747f, 14.28f, -0.28f, -3.944f},
    {1.579f, 0.888f, 5.448f, -2.436f, -7.388f, 12.0f, -0.07f, -2.78f},
    {4.454f, 1.559f, 0.5054f, 0.309f, 4.838f, 14.27f, -4.384f, -4.654f},
    {7.651f, -3.82f, 5.1581f, 0.384f, -4.809f, 21.86f, -2.905f, 0.745f},
    {-0.458f, -2.523f, 6.1159f, 0.686f, -2.951f, 15.4f, -2.125f, -0.736f},
    {5.416f, 1.513f, 4.7725f, 3.673f, 5.573f, 22.87f, 3.273f, 2.217f},
    {-3.061f, -7.838f, 6.1486f, -0.707f, 6.013f, 21.88f, 3.974f, -2.714f},
    {0.32f, -3.442f, 4.2501f, -0.191f, -3.898f, 14.94f, -4.504f, 2.617f},
    {-6.391f, -6.689f, 5.6249f, -4.917f, -2.818f, 18.51f, 3.504f, -1.08f},
    {-5.931f, 6.447f, 5.01f, -3.364f, 1.727f, 19.54f, 4.131f, -0.36f},
    {-5.171f, 5.447f, 3.6329f, -1.988f, -5.81f, 18.66f, 2.379f, 0.497f},
    {-8.762f, -6.084f, 4.2433f, 1.182f, 7.649f, 17.71f, 3.418f, 0.15f},
    {-5.86f, 9.443f, 5.2591f, -4.844f, 2.863f, 12.43f, 1.982f, -4.03f},
    {-9.196f, 1.835f, 1.4442f, -4.829f, 4.039f, 23.04f, 3.682f, 1.169f},
    {-5.146f, 0.286f, 5.3835f, 4.434f, 4.819f, 12.18f, -0.084f, -2.145f},
    {-1.859f, -6.128f, 1.2728f, -1.363f, -1.489f, 16.12f, 4.879f, 3.061f},
    {0.79f, 4.901f, 4.6727f, -3.387f, 3.229f, 20.04f, -3.768f, -0.687f},
    {-2.455f, 5.822f, 5.0368f, 2.318f, -9.748f, 12.15f, -1.825f, -3.32f},
    {7.243f, -4.918f, 0.9621f, 1.459f, 1.649f, 15.38f, 0.332f, -3.844f},
    {-2.269f, -9.972f, 4.3379f, -3.803f, 6.33f, 15.92f, -1.336f, -0.925f},
    {7.879f, 0.086f, 4.2353f, 1.129f, 6.396f, 12.35f, -1.833f, -4.499f},
    {2.024f, 1.991f, 4.15f, -4.057f, -5.744f, 14.29f, -0.545f, -4.509f},
    {-8.296f, -7.03f, 5.0718f, -3.438f, 8.644f, 12.17f, 3.63f, 1.338f},
    {-0.051f, -7.68f, 2.2896f, 0.446f, -9.511f, 16.28f, 3.754f, 2.504f},
    {-5.048f, 0.408f, 1.3054f, -4.128f, 1.596f, 12.1f, 2.822f, -4.36f},
    {5.345f, -8.204f, 0.4383f, -3.424f, 9.901f, 20.83f, -1.939f, -4.915f},
    {3.414f, 5.318f, 5.2545f, -2.394f, -3.621f, 10.76f, 3.746f, -2.35f},
    {-2.523f, 6.282f, 5.0548f, 0.266f, -7.948f, 16.14f, 3.257f, 3.059f},
    {3.897f, 0.73f, 2.0929f, 2.597f, -9.352f, 14.1f, -3.741f, 2.573f},
    {3.826f, 4.166f, 5.2611f, -4.646f, 3.998f, 12.99f, -1.25f, 2.584f},
    {-5.85f, 5.014f, 3.4521f, 4.875f, 4.592f, 18.09f, -2.756f, -4.696f},
    {7.924f, -9.118f, 4.4649f, 3.368f, -9.909f, 19.59f, 4.375f, -1.904f},
    {0.476f, 9.664f, 1.4493f, 2.997f, -3.938f, 21.82f, 0.818f, 3.645f},
    {-2.246f, -6.877f, 1.9548f, 0.818f, -3.136f, 23.68f, 4.402f, -2.944f},
    {-0.086f, 0.329f, 5.8108f, 4.962f, -1.379f, 17.43f, 2.362f, 4.278f},
    {8.244f, -7.339f, 2.8767f, -1.833f, -7.874f, 18.76f, -4.836f, -3.157f},
    {-0.774f, 6.503f, 3.0452f, -2.281f, 3.723f, 18.9f, 1.83f, -3.159f},
};

void foc_vector_step(const foc_vector_in_t* in, foc_vector_out_t* out) {
    foc_abc_t i_abc = {.a = in->ia, .b = -(in->ia + in->ic), .c = in->ic};
    foc_sincos_t sc = {.sin_theta = sinf(in->theta), .cos_theta = cosf(in->theta)};
    float v_lim = in->vbus * FOC_VECTORS_INV_SQRT3;
    foc_pi_t pi_d = {
        .kp = FOC_VECTORS_KP, .ki = FOC_VECTORS_KI, .out_min = -v_lim, .out_max = v_lim, .integral = in->int_d};
    foc_pi_t pi_q = {
        .kp = FOC_VECTORS_KP, .ki = FOC_VECTORS_KI, .out_min = -v_lim, .out_max = v_lim, .integral = in->int_q};

    out->i_dq = foc_park(foc_clarke(i_abc), sc);
    out->v_dq.d = foc_pi_step(&pi_d, in->id_ref - out->i_dq.d, FOC_VECTORS_DT_S);
    out->v_dq.q = foc_pi_step(&pi_q, in->iq_ref - out->i_dq.q, FOC_VECTORS_DT_S);
    out->v_int.d = pi_d.integral;
    out->v_int.q = pi_q.integral;
    out->limited = foc_svpwm(foc_inv_park(out->v_dq, sc), in->vbus, &out->duty);
}

void foc_vector_fields(const foc_vector_out_t* out, float* fields) {
    fields[0] = out->i_dq.d;
    fields[1] = out->i_dq.q;
    fields[2] = out->v_dq.d;
    fields[3] = out->v_dq.q;
    fields[4] = out->v_int.d;
    fields[5] = out->v_int.q;
    fields[6] = out->duty.a;
    fields[7] = out->duty.b;
    fields[8] = out->duty.c;
}
