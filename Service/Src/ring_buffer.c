/**
  * @file    ring_buffer.c
  * @brief   Platform-generic array-based ring buffer
  */

#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb, rb_elem_t *buf, rb_idx_t capacity)
{
    rb->buffer   = buf;
    rb->capacity = capacity;
    rb->head     = 0;
    rb->tail     = 0;
    rb->count    = 0;
}

void RingBuffer_Reset(RingBuffer_t *rb)
{
    rb->head  = 0;
    rb->tail  = 0;
    rb->count = 0;
}

int RingBuffer_Write(RingBuffer_t *rb, rb_elem_t data)
{
    if (rb == (void *)0 || rb->buffer == (void *)0)
        return RB_ERR_PARAM;

    if (rb->count >= rb->capacity)
        return RB_ERR_FULL;

    rb->buffer[rb->head] = data;
    rb->head++;
    if (rb->head >= rb->capacity)
        rb->head = 0;
    rb->count++;
    return RB_OK;
}

rb_idx_t RingBuffer_WriteBlock(RingBuffer_t *rb, const rb_elem_t *data, rb_idx_t len)
{
    rb_idx_t i;
    rb_idx_t free;

    if (rb == (void *)0 || rb->buffer == (void *)0 || data == (void *)0)
        return 0;

    free = rb->capacity - rb->count;
    if (len > free)
        len = free;

    for (i = 0; i < len; i++)
    {
        rb->buffer[rb->head] = data[i];
        rb->head++;
        if (rb->head >= rb->capacity)
            rb->head = 0;
    }
    rb->count += len;
    return len;
}

rb_elem_t *RingBuffer_WriteAcquire(RingBuffer_t *rb, rb_idx_t *max_len)
{
    rb_idx_t contig;
    rb_idx_t free;

    if (rb == (void *)0 || rb->buffer == (void *)0 || max_len == (void *)0)
    {
        if (max_len) *max_len = 0;
        return (void *)0;
    }

    free = rb->capacity - rb->count;
    if (free == 0)
    {
        *max_len = 0;
        return (void *)0;
    }

    /* Contiguous space from head to end-of-buffer (or free, whichever smaller) */
    contig = rb->capacity - rb->head;
    if (contig > free)
        contig = free;

    *max_len = contig;
    return &rb->buffer[rb->head];
}

int RingBuffer_WriteCommit(RingBuffer_t *rb, rb_idx_t actual_len)
{
    if (rb == (void *)0)
        return RB_ERR_PARAM;

    if (actual_len == 0)
        return RB_OK;

    if (actual_len > (rb->capacity - rb->count))
        return RB_ERR_FULL;

    rb->head += actual_len;
    if (rb->head >= rb->capacity)
        rb->head -= rb->capacity;
    rb->count += actual_len;
    return RB_OK;
}

int RingBuffer_IsFull(const RingBuffer_t *rb)
{
    return (rb->count >= rb->capacity) ? 1 : 0;
}

int RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return (rb->count == 0) ? 1 : 0;
}

rb_idx_t RingBuffer_Available(const RingBuffer_t *rb)
{
    return rb->count;
}

rb_idx_t RingBuffer_FreeSpace(const RingBuffer_t *rb)
{
    return rb->capacity - rb->count;
}
