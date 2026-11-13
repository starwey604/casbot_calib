#include "fake_zephyr.h"
#include "qas_client.h"
#include "usb_transport.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

uint64_t fake_clock;
const struct bt_uuid_16 fake_ccc = BT_UUID_INIT_16(0x2902);
extern struct bt_conn_cb connection_callbacks;
static struct bt_conn pool[3];
static struct bt_gatt_discover_params *discoveries[3];
static struct bt_gatt_subscribe_params *subscriptions[3];
static struct bt_gatt_write_params *requests[3];
static void (*completions[3])(struct bt_conn *,void *);
static void *completion_users[3];
static unsigned int creates, disconnects, writes;
static uint16_t written_handle;
static uint32_t tx_drops, rx_drops;

int bt_enable(void *cb) { (void)cb; return 0; }
int bt_addr_from_str(const char *s,bt_addr_t *out)
{
    unsigned int n[6];
    if(strlen(s)!=17 || sscanf(s,"%x:%x:%x:%x:%x:%x",&n[0],&n[1],&n[2],&n[3],&n[4],&n[5])!=6)return -EINVAL;
    for(int i=0;i<6;i++){out->val[i]=(uint8_t)n[i];}
    return 0;
}
void bt_addr_le_to_str(const bt_addr_le_t *a,char *out,size_t size)
{ (void)a; snprintf(out,size,"78:CD:55:45:67:AD (public)"); }
int bt_conn_le_create(const bt_addr_le_t *a,const void *p,const void *q,struct bt_conn **out)
{
    (void)a;(void)p;(void)q;
    for(int i=0;i<3;i++)if(!pool[i].refs){pool[i]=(struct bt_conn){i,1};*out=&pool[i];creates++;return 0;}
    return -ENOMEM;
}
int bt_conn_disconnect(struct bt_conn *c,uint8_t reason)
{ disconnects++; connection_callbacks.disconnected(c,reason);return 0; }
void bt_conn_unref(struct bt_conn *c){assert(c->refs==1);c->refs=0;}
int bt_conn_get_info(struct bt_conn *c,struct bt_conn_info *out)
{ (void)c;out->le.interval_us=30000;out->le.latency=0;out->le.timeout=400;return 0; }
int bt_le_scan_start(const void *p,void (*cb)(const bt_addr_le_t *,int8_t,uint8_t,struct net_buf_simple *))
{(void)p;(void)cb;return 0;}
int bt_le_scan_stop(void){return 0;}
int bt_gatt_discover(struct bt_conn *c,struct bt_gatt_discover_params *p){discoveries[c->index]=p;return 0;}
int bt_gatt_subscribe(struct bt_conn *c,struct bt_gatt_subscribe_params *p){subscriptions[c->index]=p;return 0;}
uint16_t bt_gatt_get_mtu(struct bt_conn *c){(void)c;return 23;}
int bt_gatt_write(struct bt_conn *c,struct bt_gatt_write_params *p)
{ requests[c->index]=p;written_handle=p->handle;writes++;return 0; }
int bt_gatt_write_without_response_cb(struct bt_conn *c,uint16_t h,const void *data,uint16_t len,bool sign,
                                     void (*done)(struct bt_conn *,void *),void *user)
{
    assert(!sign && len && ((const char *)data)[len]=='\0');
    written_handle=h;writes++;completions[c->index]=done;completion_users[c->index]=user;return 0;
}

/* Compile the actual USB command owner with deterministic transport substitutes. */
int bridge_usb_init(void){return 0;}
bool bridge_usb_connected(void){return true;}
bool bridge_usb_line(char *line){(void)line;return false;}
void bridge_usb_reset(void){}
uint32_t bridge_usb_tx_drops(void){return tx_drops;}
uint32_t bridge_usb_rx_drops(void){return rx_drops;}
static unsigned int json_count;
int bridge_usb_json(const char *format,...)
{
    char line[USB_JSON_MAX];va_list args;va_start(args,format);
    int n=vsnprintf(line,sizeof(line),format,args);va_end(args);
    assert(n>0 && n<(int)sizeof(line)-1);
    puts(line);json_count++;return 0;
}
#include "../../../trical_bridge/lib/bridge_app.c"

static void pump(void)
{
    struct qas_event e;
    for(int i=0;i<1000 && qas_next_event(&e);i++){qas_process(&e);event_output(&e);bench_event(&e);}
}
static void attribute(int index,uint16_t handle,void *user)
{
    struct bt_gatt_attr attr={handle,user};
    struct bt_gatt_discover_params *d=discoveries[index];
    d->func(&pool[index],handle?&attr:NULL,d);
}
static void make_ready(unsigned int slot,const char *address,bool variant128)
{
    assert(qas_connect(slot,address,"public")==0);
    int index=-1;for(int i=0;i<3;i++)if(pool[i].refs && discoveries[i]==NULL)index=i;
    assert(index>=0);
    connection_callbacks.connected(&pool[index],0);pump();
    assert(!qas_ready(slot));
    if(variant128){attribute(index,0,NULL);pump();assert(discoveries[index]->uuid->type==128);}
    struct bt_gatt_service_val service={30};attribute(index,10,&service);pump();
    assert(discoveries[index]->type==BT_GATT_DISCOVER_CHARACTERISTIC);
    static const struct bt_uuid_16 uuid16=BT_UUID_INIT_16(0xffb2);
    static const struct bt_uuid_128 uuid128=BT_UUID_INIT_128(0xa304d249ffb2);
    struct bt_gatt_chrc ch={variant128?&uuid128.uuid:&uuid16.uuid,12,0x1c};
    attribute(index,11,&ch);
    ch.uuid=&fake_ccc.uuid;ch.value_handle=21;attribute(index,20,&ch);pump();
    assert(discoveries[index]->start_handle==13 && discoveries[index]->end_handle==19);
    attribute(index,13,NULL);pump();
    assert(!qas_ready(slot)); /* The subscribe API returning zero is not a CCC acknowledgment. */
    assert(subscriptions[index]->value_handle==12 && subscriptions[index]->ccc_handle==13);
    subscriptions[index]->subscribe(&pool[index],0,subscriptions[index]);pump();assert(qas_ready(slot));
}
static void packet(unsigned int index,const char *text)
{
    subscriptions[index]->notify(&pool[index],subscriptions[index],text,strlen(text));pump();
}

int main(void)
{
    assert(qas_client_init()==0);
    assert(qas_connect(0,"78:CD:55:45:67:AD","guess")==-EINVAL);
    make_ready(0,"78:CD:55:45:67:AD",false);
    assert(qas_connect(1,"78:CD:55:45:67:AD","public")==-EALREADY);
    make_ready(1,"78:CD:55:45:67:AE",true);
    make_ready(2,"78:CD:55:45:67:AF",false);
    assert(qas_ready(0)&&qas_ready(1)&&qas_ready(2));
    assert(qas_send(0,1,"OTA",false)==-EINVAL && writes==0);
    assert(qas_send(0,2,"MEAS",false)==0 && written_handle==12);
    assert(qas_send(0,3,"MOD",false)==-EBUSY);
    completions[0](&pool[0],completion_users[0]);pump();assert(qas_idle(0));
    assert(qas_send(1,4,"INT",true)==0);requests[1]->func(&pool[1],0,requests[1]);pump();
    packet(0,"MOD:0\r\n");assert(qas_mod(0)==0);
    packet(0,"DAT1:m+0.000");fake_clock=2000000;
    char denied[]="9 rate 0 100 1000 500 wcmd";command(denied);assert(!bench.active);
    bench.enabled=true;
    char cmd[]="10 rate 0 100 1000 500 wcmd";command(cmd);assert(bench.active);
    struct qas_event data={.type=QAS_NOTIFY,.slot=0,.epoch=qas_epoch(0),.length=244,.time_us=2000000};
    memset(data.data,'?',244);event_output(&data); /* Maximum raw JSON line fits. */
    packet(0,"DAT0:1.000");packet(0,"DAT1:1.000");
    assert(bench.core.data[0].count==1&&bench.core.data[1].count==1);
    char long_command[]="11 cmd 0 wcmd OTA extra";command(long_command);assert(writes==2);
    stop_bench("cancelled");assert(!bench.active);
    uint32_t frozen=bench.end_tx;tx_drops++;report();assert(bench.end_tx==frozen);
    assert(qas_send(0,5,"MEAS",false)==0);
    assert(qas_disconnect(0)==0);pump();assert(!qas_ready(0));
    assert(qas_send(0,6,"MEAS",false)==-ENOTCONN);
    assert(creates==3); /* Never auto-reconnect. */
    assert(qas_scan(100)==0);fake_clock+=100000;qas_tick();pump();
    /* Dropped control events are terminal; a dropped raw packet is counted. */
    for(int i=0;i<100;i++)subscriptions[1]->notify(&pool[1],subscriptions[1],"DAT0:1.0",8);
    connection_callbacks.le_param_updated(&pool[1],24,0,400);
    assert(qas_event_drops()>0);qas_tick();assert(qas_terminal());pump();
    assert(qas_connect(0,"78:CD:55:45:67:AD","public")==-EIO);
    assert(json_count>=5&&disconnects>=3);
    fprintf(stderr,"PASS: real QAS client/application with fake GATT: 3 slots, 16/128-bit discovery, CCC ack, writes, faults, queues and JSON bounds\n");
    return 0;
}
