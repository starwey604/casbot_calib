#ifndef TRICAL_FAKE_ZEPHYR_H
#define TRICAL_FAKE_ZEPHYR_H
/* Deterministic GATT/kernel substitutes for testing the actual client/application C. */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define ARG_UNUSED(x) (void)(x)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define CONTAINER_OF(p,t,f) ((t *)((char *)(p)-offsetof(t,f)))
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000000
#define K_NO_WAIT 0
extern uint64_t fake_clock;
static inline uint64_t k_uptime_ticks(void) { return fake_clock; }
static inline uint64_t k_ticks_to_us_floor64(uint64_t t) { return t; }
static inline void k_msleep(int ms) { fake_clock += ms * 1000; }
typedef int atomic_t;
static inline int atomic_inc(atomic_t *v) { return (*v)++; }
static inline int atomic_get(atomic_t *v) { return *v; }
static inline void atomic_set(atomic_t *v, int n) { *v=n; }
static inline void atomic_set_bit(atomic_t *v, int n) { *v |= 1<<n; }
struct k_msgq { unsigned char *data; size_t item, capacity, begin, count; };
#define K_MSGQ_DEFINE(name,sz,cap,align) \
    static unsigned char name##_storage[(sz)*(cap)]; \
    struct k_msgq name={.data=name##_storage,.item=(sz),.capacity=(cap)}
static inline int k_msgq_put(struct k_msgq *q,const void *v,int wait)
{
    (void)wait; if(q->count==q->capacity)return -ENOBUFS;
    memcpy(q->data+((q->begin+q->count)%q->capacity)*q->item,v,q->item);q->count++;return 0;
}
static inline int k_msgq_get(struct k_msgq *q,void *v,int wait)
{
    (void)wait;if(!q->count)return -ENOMSG;
    memcpy(v,q->data+q->begin*q->item,q->item);q->begin=(q->begin+1)%q->capacity;q->count--;return 0;
}
static inline unsigned int k_msgq_num_used_get(struct k_msgq *q){return q->count;}
static inline void k_msgq_purge(struct k_msgq *q){q->begin=0;q->count=0;}
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
static inline int k_spin_lock(struct k_spinlock *lock){(void)lock;return 0;}
static inline void k_spin_unlock(struct k_spinlock *lock,int key){(void)lock;(void)key;}
struct bt_uuid { unsigned int type; };
struct bt_uuid_16 { struct bt_uuid uuid; uint16_t val; };
struct bt_uuid_128 { struct bt_uuid uuid; uint64_t val; };
#define BT_UUID_INIT_16(v) {.uuid={16},.val=(v)}
#define BT_UUID_128_ENCODE(a,b,c,d,e) (e)
#define BT_UUID_INIT_128(v) {.uuid={128},.val=(v)}
extern const struct bt_uuid_16 fake_ccc;
#define BT_UUID_GATT_CCC (&fake_ccc.uuid)
static inline int bt_uuid_cmp(const struct bt_uuid *a,const struct bt_uuid *b)
{
    if(a->type!=b->type)return 1;
    if(a->type==16)return ((const struct bt_uuid_16 *)a)->val!=((const struct bt_uuid_16 *)b)->val;
    return ((const struct bt_uuid_128 *)a)->val!=((const struct bt_uuid_128 *)b)->val;
}
typedef struct { uint8_t val[6]; } bt_addr_t;
typedef struct { uint8_t type; bt_addr_t a; } bt_addr_le_t;
#define BT_ADDR_LE_PUBLIC 0
#define BT_ADDR_LE_RANDOM 1
int bt_addr_from_str(const char *,bt_addr_t *);
static inline int bt_addr_le_cmp(const bt_addr_le_t *a,const bt_addr_le_t *b){return memcmp(a,b,sizeof(*a));}
void bt_addr_le_to_str(const bt_addr_le_t *,char *,size_t);
struct bt_conn { int index, refs; };
struct bt_conn_cb {
    void (*connected)(struct bt_conn *,uint8_t);
    void (*disconnected)(struct bt_conn *,uint8_t);
    void (*le_param_updated)(struct bt_conn *,uint16_t,uint16_t,uint16_t);
};
#define BT_CONN_CB_DEFINE(name) struct bt_conn_cb name
struct bt_conn_info { struct { uint32_t interval_us;uint16_t latency,timeout; } le; };
#define BT_CONN_LE_CREATE_CONN NULL
#define BT_LE_CONN_PARAM_DEFAULT NULL
#define BT_HCI_ERR_REMOTE_USER_TERM_CONN 0x13
int bt_enable(void *);
int bt_conn_le_create(const bt_addr_le_t *,const void *,const void *,struct bt_conn **);
int bt_conn_disconnect(struct bt_conn *,uint8_t);
void bt_conn_unref(struct bt_conn *);
int bt_conn_get_info(struct bt_conn *,struct bt_conn_info *);
struct net_buf_simple { uint16_t len; uint8_t *data; };
#define BT_LE_SCAN_ACTIVE NULL
int bt_le_scan_start(const void *,void (*)(const bt_addr_le_t *,int8_t,uint8_t,struct net_buf_simple *));
int bt_le_scan_stop(void);
struct bt_gatt_attr { uint16_t handle; void *user_data; };
struct bt_gatt_service_val { uint16_t end_handle; };
struct bt_gatt_chrc { const struct bt_uuid *uuid; uint16_t value_handle;uint8_t properties; };
struct bt_gatt_discover_params {
    const struct bt_uuid *uuid;
    uint8_t (*func)(struct bt_conn *,const struct bt_gatt_attr *,struct bt_gatt_discover_params *);
    uint16_t start_handle,end_handle;uint8_t type;
};
struct bt_gatt_subscribe_params {
    uint8_t (*notify)(struct bt_conn *,struct bt_gatt_subscribe_params *,const void *,uint16_t);
    void (*subscribe)(struct bt_conn *,uint8_t,struct bt_gatt_subscribe_params *);
    uint16_t value_handle,ccc_handle,value; atomic_t flags[1];
};
struct bt_gatt_write_params {
    void (*func)(struct bt_conn *,uint8_t,struct bt_gatt_write_params *);
    uint16_t handle,length; const void *data;
};
#define BT_GATT_ITER_STOP 0
#define BT_GATT_ITER_CONTINUE 1
#define BT_GATT_DISCOVER_PRIMARY 1
#define BT_GATT_DISCOVER_CHARACTERISTIC 2
#define BT_GATT_DISCOVER_DESCRIPTOR 3
#define BT_GATT_CHRC_WRITE 0x08
#define BT_GATT_CHRC_WRITE_WITHOUT_RESP 0x04
#define BT_GATT_CHRC_NOTIFY 0x10
#define BT_GATT_CCC_NOTIFY 1
#define BT_GATT_SUBSCRIBE_FLAG_VOLATILE 0
int bt_gatt_discover(struct bt_conn *,struct bt_gatt_discover_params *);
int bt_gatt_subscribe(struct bt_conn *,struct bt_gatt_subscribe_params *);
uint16_t bt_gatt_get_mtu(struct bt_conn *);
int bt_gatt_write(struct bt_conn *,struct bt_gatt_write_params *);
int bt_gatt_write_without_response_cb(struct bt_conn *,uint16_t,const void *,uint16_t,bool,
                                     void (*)(struct bt_conn *,void *),void *);
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#endif
