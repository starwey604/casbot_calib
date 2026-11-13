#include "fake_zephyr.h"
struct ring_buf { uint8_t *bytes; uint32_t capacity,begin,count; };
#define RING_BUF_DECLARE(name,cap) static uint8_t name##_storage[cap]; \
    static struct ring_buf name={.bytes=name##_storage,.capacity=(cap)}
static inline uint32_t ring_buf_space_get(struct ring_buf *r){return r->capacity-r->count;}
static inline uint32_t ring_buf_put(struct ring_buf *r,const uint8_t *src,uint32_t n)
{
    n=MIN(n,ring_buf_space_get(r));
    for(uint32_t i=0;i<n;i++){r->bytes[(r->begin+r->count+i)%r->capacity]=src[i];}
    r->count+=n;return n;
}
static inline uint32_t ring_buf_get_ptr(struct ring_buf *r,uint8_t **out,size_t offset)
{
    assert(offset<=r->count);
    uint32_t pos=(r->begin+offset)%r->capacity;*out=r->bytes+pos;
    return MIN(r->count-offset,r->capacity-pos);
}
static inline void ring_buf_consume(struct ring_buf *r,size_t n)
{assert(n<=r->count);r->begin=(r->begin+n)%r->capacity;r->count-=n;}
static inline void ring_buf_reset(struct ring_buf *r){r->begin=0;r->count=0;}
