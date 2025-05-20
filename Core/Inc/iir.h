#pragma once
#include "arm_math_types.h"
#ifndef FILTER_IIR_H
#define FILTER_IIR_H

typedef struct {
    const float32_t* b; // coefficients b0 to bn, size n+1
    const float32_t* a; // coefficients a1 to an, size n
    int32_t n;
    float32_t* d; // delays (buffer), size n
} filter_iir_t;


#ifdef __cplusplus
extern "C" {
#endif

    void filter_iir_init(filter_iir_t* f,
        const float* b,
        const float* a,
        int32_t n,
        float32_t* d);

    float filter_iir_apply(filter_iir_t* f, float in);

#ifdef __cplusplus
}
#endif

#endif