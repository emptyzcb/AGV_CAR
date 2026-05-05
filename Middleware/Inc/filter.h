/**
  * @file    filter.h
  * @brief   Data filtering: Moving Average + Exponential Moving Average
  */

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* ---- Moving Average ---- */
#define FILTER_MA_MAX_SIZE  32

typedef struct {
    float   buffer[FILTER_MA_MAX_SIZE];
    uint8_t size;
    uint8_t index;
    uint8_t count;
    float   sum;
} Filter_MA_t;

void  Filter_MA_Init(Filter_MA_t *f, uint8_t size);
float Filter_MA_Update(Filter_MA_t *f, float sample);
float Filter_MA_GetValue(Filter_MA_t *f);
void  Filter_MA_Reset(Filter_MA_t *f);

/* ---- Exponential Moving Average (Low-Pass) ---- */
typedef struct {
    float    alpha;
    float    value;
    uint8_t  initialized;
} Filter_EMA_t;

void  Filter_EMA_Init(Filter_EMA_t *f, float alpha);
float Filter_EMA_Update(Filter_EMA_t *f, float sample);
float Filter_EMA_GetValue(Filter_EMA_t *f);
void  Filter_EMA_Reset(Filter_EMA_t *f);

#endif /* FILTER_H */
