/**
  * @file    filter.c
  * @brief   Moving average and EMA filter implementation
  */

#include "filter.h"

/* ---- Moving Average ---- */

void Filter_MA_Init(Filter_MA_t *f, uint8_t size)
{
    if (size > FILTER_MA_MAX_SIZE) size = FILTER_MA_MAX_SIZE;
    if (size == 0) size = 1;
    f->size  = size;
    f->index = 0;
    f->count = 0;
    f->sum   = 0.0f;
    for (uint8_t i = 0; i < FILTER_MA_MAX_SIZE; i++)
        f->buffer[i] = 0.0f;
}

float Filter_MA_Update(Filter_MA_t *f, float sample)
{
    f->sum -= f->buffer[f->index];
    f->sum += sample;
    f->buffer[f->index] = sample;
    f->index = (f->index + 1) % f->size;
    if (f->count < f->size) f->count++;
    return f->sum / f->count;
}

float Filter_MA_GetValue(Filter_MA_t *f)
{
    if (f->count == 0) return 0.0f;
    return f->sum / f->count;
}

void Filter_MA_Reset(Filter_MA_t *f)
{
    f->index = 0;
    f->count = 0;
    f->sum   = 0.0f;
}

/* ---- EMA ---- */

void Filter_EMA_Init(Filter_EMA_t *f, float alpha)
{
    f->alpha       = alpha;
    f->value       = 0.0f;
    f->initialized = 0;
}

float Filter_EMA_Update(Filter_EMA_t *f, float sample)
{
    if (!f->initialized)
    {
        f->value       = sample;
        f->initialized = 1;
    }
    else
    {
        f->value = f->alpha * sample + (1.0f - f->alpha) * f->value;
    }
    return f->value;
}

float Filter_EMA_GetValue(Filter_EMA_t *f)
{
    return f->value;
}

void Filter_EMA_Reset(Filter_EMA_t *f)
{
    f->value       = 0.0f;
    f->initialized = 0;
}
