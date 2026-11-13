#include "usb_transport.h"
#include <zephyr/device.h>
#include <stdio.h>

struct device fake_uart={1};
static bool dtr=true, tx_enabled;
static unsigned char input[4096], output[32768];
static size_t input_length, output_length;
static int tx_space;
static void (*irq)(const struct device *,void *);
void uart_irq_update(const struct device *dev){(void)dev;}
int uart_irq_is_pending(const struct device *dev){(void)dev;return input_length || (tx_enabled&&tx_space>0);}
int uart_irq_rx_ready(const struct device *dev){(void)dev;return input_length!=0;}
int uart_irq_tx_ready(const struct device *dev){(void)dev;return tx_enabled&&tx_space>0;}
int uart_fifo_read(const struct device *dev,uint8_t *dest,int size)
{
    (void)dev;size_t n=MIN((size_t)size,input_length);memcpy(dest,input,n);
    memmove(input,input+n,input_length-n);input_length-=n;return n;
}
int uart_fifo_fill(const struct device *dev,const uint8_t *data,int size)
{
    (void)dev;int n=MIN(size,tx_space);assert(output_length+n<sizeof(output));
    memcpy(output+output_length,data,n);output_length+=n;tx_space-=n;return n;
}
void uart_irq_tx_enable(const struct device *dev){(void)dev;tx_enabled=true;}
void uart_irq_tx_disable(const struct device *dev){(void)dev;tx_enabled=false;}
void uart_irq_rx_enable(const struct device *dev){(void)dev;}
int uart_line_ctrl_get(const struct device *dev,int op,uint32_t *value){(void)dev;(void)op;*value=dtr;return 0;}
int uart_irq_callback_user_data_set(const struct device *dev,void (*cb)(const struct device *,void *),void *user)
{(void)dev;(void)user;irq=cb;return 0;}
static void receive(const void *data,size_t n)
{assert(n<sizeof(input));memcpy(input,data,n);input_length=n;irq(&fake_uart,NULL);}
int main(void)
{
    assert(bridge_usb_init()==0);
    char line[USB_COMMAND_MAX];
    receive("1 status\r\n",10);assert(bridge_usb_line(line)&&!strcmp(line,"1 status"));
    char too_long[300];memset(too_long,'X',sizeof(too_long));too_long[299]='\n';
    receive(too_long,sizeof(too_long));assert(!bridge_usb_line(line)&&bridge_usb_rx_drops()==1);
    receive("2 status\n",9);assert(bridge_usb_line(line)&&!strcmp(line,"2 status"));
    const char poisoned[]={ '3',' ',0,'s','t','a','t','u','s','\n' };
    receive(poisoned,sizeof(poisoned));assert(!bridge_usb_line(line)&&bridge_usb_rx_drops()==2);
    for(int i=0;i<9;i++){receive("4 status\n",9);}
    assert(bridge_usb_rx_drops()==3);bridge_usb_reset();assert(!bridge_usb_line(line));
    assert(bridge_usb_json("{\"type\":\"status\",\"value\":%d}",123)==0);
    while(tx_enabled){tx_space=7;irq(&fake_uart,NULL);} /* Every FIFO fill is partial. */
    output[output_length]=0;
    assert(!strcmp((char *)output,"{\"type\":\"status\",\"value\":123}\n"));
    char payload[800];memset(payload,'X',799);payload[799]=0;
    int accepted=0;
    while(bridge_usb_json("%s",payload)==0){accepted++;}
    assert(accepted==20&&bridge_usb_tx_drops()==1);
    bridge_usb_reset();assert(!tx_enabled);
    dtr=false;assert(!bridge_usb_connected());
    assert(bridge_usb_json("{}")==-ENOTCONN);receive("5 status\n",9);assert(!bridge_usb_line(line));
    dtr=true;receive("6 status\n",9);assert(bridge_usb_line(line));
    puts("PASS: actual USB transport with UART/ring substitutes: partial TX, complete lines, overrun, binary poisoning and DTR reset");
    return 0;
}
