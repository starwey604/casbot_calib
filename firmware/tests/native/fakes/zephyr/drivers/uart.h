#include <zephyr/device.h>
#define UART_LINE_CTRL_DTR 1
void uart_irq_update(const struct device *);
int uart_irq_is_pending(const struct device *);
int uart_irq_rx_ready(const struct device *);
int uart_irq_tx_ready(const struct device *);
int uart_fifo_read(const struct device *,uint8_t *,int);
int uart_fifo_fill(const struct device *,const uint8_t *,int);
void uart_irq_tx_enable(const struct device *);
void uart_irq_tx_disable(const struct device *);
void uart_irq_rx_enable(const struct device *);
int uart_line_ctrl_get(const struct device *,int,uint32_t *);
int uart_irq_callback_user_data_set(const struct device *,void (*)(const struct device *,void *),void *);
