/**
  * @file    ring_buffer.h
  * @brief   Platform-generic array-based ring buffer
  */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

/*--- User configuration ---------------------------------------------------*/
/* Override these defines before include, or in your project config header. */

#ifndef RING_BUFFER_ELEM_TYPE
#define RING_BUFFER_ELEM_TYPE   uint8_t
#endif

#ifndef RING_BUFFER_INDEX_TYPE
#define RING_BUFFER_INDEX_TYPE  uint32_t
#endif

/*--- Type definitions -----------------------------------------------------*/

typedef RING_BUFFER_ELEM_TYPE   rb_elem_t;
typedef RING_BUFFER_INDEX_TYPE  rb_idx_t;

typedef struct {
    rb_elem_t  *buffer;     /* pointer to user-allocated array */
    rb_idx_t    capacity;   /* max number of elements */
    rb_idx_t    head;       /* write index */
    rb_idx_t    tail;       /* read index  */
    rb_idx_t    count;      /* current element count */
} RingBuffer_t;

/*--- Status ---------------------------------------------------------------*/
#define RB_OK           0
#define RB_ERR_FULL    -1
#define RB_ERR_EMPTY   -2
#define RB_ERR_PARAM   -3

/*--- API ------------------------------------------------------------------*/
void      RingBuffer_Init(RingBuffer_t *rb, rb_elem_t *buf, rb_idx_t capacity);
void      RingBuffer_Reset(RingBuffer_t *rb);

/* Copy-write */
int       RingBuffer_Write(RingBuffer_t *rb, rb_elem_t data);
rb_idx_t  RingBuffer_WriteBlock(RingBuffer_t *rb, const rb_elem_t *data, rb_idx_t len);

/* Zero-copy write: Acquire -> write directly -> Commit */
rb_elem_t *RingBuffer_WriteAcquire(RingBuffer_t *rb, rb_idx_t *max_len);
int        RingBuffer_WriteCommit(RingBuffer_t *rb, rb_idx_t actual_len);

/* Query */
int       RingBuffer_IsFull(const RingBuffer_t *rb);
int       RingBuffer_IsEmpty(const RingBuffer_t *rb);
rb_idx_t  RingBuffer_Available(const RingBuffer_t *rb);
rb_idx_t  RingBuffer_FreeSpace(const RingBuffer_t *rb);

#endif /* RING_BUFFER_H */
