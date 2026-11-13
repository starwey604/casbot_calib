#ifndef TRICAL_USB_TRANSPORT_H
#define TRICAL_USB_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#define USB_COMMAND_MAX 128
#define USB_JSON_MAX 1024

int bridge_usb_init(void);
bool bridge_usb_connected(void);
bool bridge_usb_line(char *line);
int bridge_usb_json(const char *format, ...);
void bridge_usb_reset(void);
uint32_t bridge_usb_tx_drops(void);
uint32_t bridge_usb_rx_drops(void);

#endif
