/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dw_apb_uart

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_dw_rockchip, CONFIG_UART_LOG_LEVEL);

/* DesignWare UART register offsets */
#define UART_RBR_THR_DLL    0x00  /* Receive Buffer/Transmit Holding/Divisor Latch Low */
#define UART_IER_DLH        0x04  /* Interrupt Enable/Divisor Latch High */
#define UART_IIR_FCR        0x08  /* Interrupt Identification/FIFO Control */
#define UART_LCR            0x0C  /* Line Control */
#define UART_MCR            0x10  /* Modem Control */
#define UART_LSR            0x14  /* Line Status */
#define UART_MSR            0x18  /* Modem Status */
#define UART_SCR            0x1C  /* Scratch */
#define UART_USR            0x7C  /* UART Status */
#define UART_TFL            0x80  /* Transmit FIFO Level */
#define UART_RFL            0x84  /* Receive FIFO Level */

/* Line Control Register bits */
#define UART_LCR_WLS_MSK    0x03  /* Word Length Select mask */
#define UART_LCR_WLS_5      0x00  /* 5 bits */
#define UART_LCR_WLS_6      0x01  /* 6 bits */
#define UART_LCR_WLS_7      0x02  /* 7 bits */
#define UART_LCR_WLS_8      0x03  /* 8 bits */
#define UART_LCR_STB        0x04  /* Stop Bit */
#define UART_LCR_PEN        0x08  /* Parity Enable */
#define UART_LCR_EPS        0x10  /* Even Parity Select */
#define UART_LCR_STKP       0x20  /* Stick Parity */
#define UART_LCR_SBRK       0x40  /* Set Break */
#define UART_LCR_BKSE       0x80  /* Bank Select Enable */
#define UART_LCR_DLAB       0x80  /* Divisor Latch Access Bit */

/* Line Status Register bits */
#define UART_LSR_DR         0x01  /* Data Ready */
#define UART_LSR_OE         0x02  /* Overrun Error */
#define UART_LSR_PE         0x04  /* Parity Error */
#define UART_LSR_FE         0x08  /* Framing Error */
#define UART_LSR_BI         0x10  /* Break Interrupt */
#define UART_LSR_THRE       0x20  /* Transmit Holding Register Empty */
#define UART_LSR_TEMT       0x40  /* Transmitter Empty */
#define UART_LSR_ERR        0x80  /* Error in RX FIFO */

/* Interrupt Enable Register bits */
#define UART_IER_ERBFI      0x01  /* Enable Received Data Available Interrupt */
#define UART_IER_ETBEI      0x02  /* Enable Transmit Holding Register Empty Interrupt */
#define UART_IER_ELSI       0x04  /* Enable Receiver Line Status Interrupt */
#define UART_IER_EDSSI      0x08  /* Enable Modem Status Interrupt */

/* Interrupt Identification Register bits */
#define UART_IIR_NO_INT     0x01  /* No interrupts pending */
#define UART_IIR_ID         0x0e  /* Interrupt ID mask */
#define UART_IIR_MSI        0x00  /* Modem status interrupt */
#define UART_IIR_THRI       0x02  /* Transmitter holding register empty */
#define UART_IIR_RDI        0x04  /* Receiver data interrupt */
#define UART_IIR_RLSI       0x06  /* Receiver line status interrupt */
#define UART_IIR_CTI        0x0c  /* Character timeout indication */

/* FIFO Control Register bits */
#define UART_FCR_FIFOE      0x01  /* FIFO Enable */
#define UART_FCR_RFIFOR     0x02  /* RX FIFO Reset */
#define UART_FCR_TFIFOR     0x04  /* TX FIFO Reset */
#define UART_FCR_DMS        0x08  /* DMA Mode Select */
#define UART_FCR_RT_MSK     0xc0  /* Receive Trigger Level mask */
#define UART_FCR_RT_1       0x00  /* 1 character */
#define UART_FCR_RT_QUARTER 0x40  /* FIFO 1/4 full */
#define UART_FCR_RT_HALF    0x80  /* FIFO 1/2 full */
#define UART_FCR_RT_ALMOST  0xc0  /* FIFO 2 less than full */

/* UART Status Register bits */
#define UART_USR_BUSY       0x01  /* UART Busy */
#define UART_USR_TFNF       0x02  /* Transmit FIFO Not Full */
#define UART_USR_TFE        0x04  /* Transmit FIFO Empty */
#define UART_USR_RFNE       0x08  /* Receive FIFO Not Empty */
#define UART_USR_RFF        0x10  /* Receive FIFO Full */

/* Default FIFO size */
#define UART_FIFO_SIZE      32

struct uart_dw_rockchip_config {
    uintptr_t reg_base;
    uint32_t sys_clk_freq;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    const struct reset_dt_spec reset_spec;
    const struct pinctrl_dev_config *pcfg;
    uint8_t line_ctrl;
    uint8_t fifo_size;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_config_func_t irq_config_func;
#endif
};

struct uart_dw_rockchip_data {
    uint32_t baud_rate;
    struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t callback;
    void *cb_data;
#endif
};

static inline uint32_t uart_dw_rockchip_read(const struct device *dev, uint32_t offset)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    return sys_read32(config->reg_base + offset);
}

static inline void uart_dw_rockchip_write(const struct device *dev, uint32_t offset, uint32_t value)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    sys_write32(value, config->reg_base + offset);
}

static void uart_dw_rockchip_set_baudrate(const struct device *dev, uint32_t baud_rate)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    uint32_t divisor;
    uint8_t lcr;

    if (baud_rate == 0) {
        return;
    }

    divisor = (config->sys_clk_freq / (16 * baud_rate));

    /* Save current LCR */
    lcr = uart_dw_rockchip_read(dev, UART_LCR);

    /* Set DLAB bit to access divisor registers */
    uart_dw_rockchip_write(dev, UART_LCR, lcr | UART_LCR_DLAB);

    /* Set divisor */
    uart_dw_rockchip_write(dev, UART_RBR_THR_DLL, divisor & 0xff);
    uart_dw_rockchip_write(dev, UART_IER_DLH, (divisor >> 8) & 0xff);

    /* Restore LCR */
    uart_dw_rockchip_write(dev, UART_LCR, lcr);
}

static int uart_dw_rockchip_configure(const struct device *dev,
                                     const struct uart_config *cfg)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    struct uart_dw_rockchip_data *data = dev->data;
    uint8_t lcr = 0;
    k_spinlock_key_t key;

    /* Configure data bits */
    switch (cfg->data_bits) {
    case UART_CFG_DATA_BITS_5:
        lcr |= UART_LCR_WLS_5;
        break;
    case UART_CFG_DATA_BITS_6:
        lcr |= UART_LCR_WLS_6;
        break;
    case UART_CFG_DATA_BITS_7:
        lcr |= UART_LCR_WLS_7;
        break;
    case UART_CFG_DATA_BITS_8:
        lcr |= UART_LCR_WLS_8;
        break;
    default:
        return -ENOTSUP;
    }

    /* Configure stop bits */
    switch (cfg->stop_bits) {
    case UART_CFG_STOP_BITS_1:
        break;
    case UART_CFG_STOP_BITS_2:
        lcr |= UART_LCR_STB;
        break;
    default:
        return -ENOTSUP;
    }

    /* Configure parity */
    switch (cfg->parity) {
    case UART_CFG_PARITY_NONE:
        break;
    case UART_CFG_PARITY_ODD:
        lcr |= UART_LCR_PEN;
        break;
    case UART_CFG_PARITY_EVEN:
        lcr |= UART_LCR_PEN | UART_LCR_EPS;
        break;
    case UART_CFG_PARITY_MARK:
        lcr |= UART_LCR_PEN | UART_LCR_STKP;
        break;
    case UART_CFG_PARITY_SPACE:
        lcr |= UART_LCR_PEN | UART_LCR_EPS | UART_LCR_STKP;
        break;
    default:
        return -ENOTSUP;
    }

    /* Configure flow control */
    if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
        return -ENOTSUP;
    }

    key = k_spin_lock(&data->lock);

    /* Set line control register */
    uart_dw_rockchip_write(dev, UART_LCR, lcr);

    /* Set baud rate */
    uart_dw_rockchip_set_baudrate(dev, cfg->baudrate);
    data->baud_rate = cfg->baudrate;

    k_spin_unlock(&data->lock, key);

    return 0;
}

static int uart_dw_rockchip_config_get(const struct device *dev,
                                      struct uart_config *cfg)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    struct uart_dw_rockchip_data *data = dev->data;
    uint8_t lcr;

    cfg->baudrate = data->baud_rate;
    cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    lcr = uart_dw_rockchip_read(dev, UART_LCR);

    /* Get data bits */
    switch (lcr & UART_LCR_WLS_MSK) {
    case UART_LCR_WLS_5:
        cfg->data_bits = UART_CFG_DATA_BITS_5;
        break;
    case UART_LCR_WLS_6:
        cfg->data_bits = UART_CFG_DATA_BITS_6;
        break;
    case UART_LCR_WLS_7:
        cfg->data_bits = UART_CFG_DATA_BITS_7;
        break;
    case UART_LCR_WLS_8:
        cfg->data_bits = UART_CFG_DATA_BITS_8;
        break;
    }

    /* Get stop bits */
    if (lcr & UART_LCR_STB) {
        cfg->stop_bits = UART_CFG_STOP_BITS_2;
    } else {
        cfg->stop_bits = UART_CFG_STOP_BITS_1;
    }

    /* Get parity */
    if (lcr & UART_LCR_PEN) {
        if (lcr & UART_LCR_EPS) {
            if (lcr & UART_LCR_STKP) {
                cfg->parity = UART_CFG_PARITY_SPACE;
            } else {
                cfg->parity = UART_CFG_PARITY_EVEN;
            }
        } else {
            if (lcr & UART_LCR_STKP) {
                cfg->parity = UART_CFG_PARITY_MARK;
            } else {
                cfg->parity = UART_CFG_PARITY_ODD;
            }
        }
    } else {
        cfg->parity = UART_CFG_PARITY_NONE;
    }

    return 0;
}

static int uart_dw_rockchip_poll_in(const struct device *dev, unsigned char *c)
{
    uint32_t lsr;

    lsr = uart_dw_rockchip_read(dev, UART_LSR);
    if (!(lsr & UART_LSR_DR)) {
        return -1;
    }

    *c = uart_dw_rockchip_read(dev, UART_RBR_THR_DLL);

    return 0;
}

static void uart_dw_rockchip_poll_out(const struct device *dev, unsigned char c)
{
    while (!(uart_dw_rockchip_read(dev, UART_LSR) & UART_LSR_THRE)) {
        /* Wait for transmitter to be ready */
    }

    uart_dw_rockchip_write(dev, UART_RBR_THR_DLL, c);
}

static int uart_dw_rockchip_err_check(const struct device *dev)
{
    uint32_t lsr;
    int errors = 0;

    lsr = uart_dw_rockchip_read(dev, UART_LSR);

    if (lsr & UART_LSR_OE) {
        errors |= UART_ERROR_OVERRUN;
    }

    if (lsr & UART_LSR_PE) {
        errors |= UART_ERROR_PARITY;
    }

    if (lsr & UART_LSR_FE) {
        errors |= UART_ERROR_FRAMING;
    }

    if (lsr & UART_LSR_BI) {
        errors |= UART_ERROR_NOISE;
    }

    return errors;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_dw_rockchip_configure_runtime(const struct device *dev,
                                             const struct uart_config *cfg)
{
    return uart_dw_rockchip_configure(dev, cfg);
}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_dw_rockchip_fifo_fill(const struct device *dev,
                                     const uint8_t *tx_data,
                                     int size)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t tfl;
    int tx_count = 0;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);

    /* Get current transmit FIFO level */
    tfl = uart_dw_rockchip_read(dev, UART_TFL);

    /* Fill FIFO until full or no more data */
    while ((tfl < config->fifo_size) && (tx_count < size)) {
        uart_dw_rockchip_write(dev, UART_RBR_THR_DLL, tx_data[tx_count]);
        tx_count++;
        tfl++;
    }

    k_spin_unlock(&data->lock, key);

    return tx_count;
}

static int uart_dw_rockchip_fifo_read(const struct device *dev,
                                     uint8_t *rx_data,
                                     const int size)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t lsr;
    int rx_count = 0;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);

    /* Read data while available and buffer space exists */
    while (rx_count < size) {
        lsr = uart_dw_rockchip_read(dev, UART_LSR);
        if (!(lsr & UART_LSR_DR)) {
            break;
        }
        rx_data[rx_count] = uart_dw_rockchip_read(dev, UART_RBR_THR_DLL);
        rx_count++;
    }

    k_spin_unlock(&data->lock, key);

    return rx_count;
}

static void uart_dw_rockchip_irq_tx_enable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier |= UART_IER_ETBEI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static void uart_dw_rockchip_irq_tx_disable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier &= ~UART_IER_ETBEI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static int uart_dw_rockchip_irq_tx_ready(const struct device *dev)
{
    return !!(uart_dw_rockchip_read(dev, UART_LSR) & UART_LSR_THRE);
}

static void uart_dw_rockchip_irq_rx_enable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier |= UART_IER_ERBFI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static void uart_dw_rockchip_irq_rx_disable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier &= ~UART_IER_ERBFI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static int uart_dw_rockchip_irq_tx_complete(const struct device *dev)
{
    return !!(uart_dw_rockchip_read(dev, UART_LSR) & UART_LSR_TEMT);
}

static int uart_dw_rockchip_irq_rx_ready(const struct device *dev)
{
    return !!(uart_dw_rockchip_read(dev, UART_LSR) & UART_LSR_DR);
}

static void uart_dw_rockchip_irq_err_enable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier |= UART_IER_ELSI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static void uart_dw_rockchip_irq_err_disable(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;
    uint32_t ier;
    k_spinlock_key_t key;

    key = k_spin_lock(&data->lock);
    ier = uart_dw_rockchip_read(dev, UART_IER_DLH);
    ier &= ~UART_IER_ELSI;
    uart_dw_rockchip_write(dev, UART_IER_DLH, ier);
    k_spin_unlock(&data->lock, key);
}

static int uart_dw_rockchip_irq_is_pending(const struct device *dev)
{
    return !(uart_dw_rockchip_read(dev, UART_IIR_FCR) & UART_IIR_NO_INT);
}

static int uart_dw_rockchip_irq_update(const struct device *dev)
{
    return 1;
}

static void uart_dw_rockchip_irq_callback_set(const struct device *dev,
                                             uart_irq_callback_user_data_t cb,
                                             void *cb_data)
{
    struct uart_dw_rockchip_data *data = dev->data;

    data->callback = cb;
    data->cb_data = cb_data;
}

static void uart_dw_rockchip_isr(const struct device *dev)
{
    struct uart_dw_rockchip_data *data = dev->data;

    if (data->callback) {
        data->callback(dev, data->cb_data);
    }
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_dw_rockchip_driver_api = {
    .poll_in = uart_dw_rockchip_poll_in,
    .poll_out = uart_dw_rockchip_poll_out,
    .err_check = uart_dw_rockchip_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_dw_rockchip_configure_runtime,
#endif
    .config_get = uart_dw_rockchip_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_dw_rockchip_fifo_fill,
    .fifo_read = uart_dw_rockchip_fifo_read,
    .irq_tx_enable = uart_dw_rockchip_irq_tx_enable,
    .irq_tx_disable = uart_dw_rockchip_irq_tx_disable,
    .irq_tx_ready = uart_dw_rockchip_irq_tx_ready,
    .irq_rx_enable = uart_dw_rockchip_irq_rx_enable,
    .irq_rx_disable = uart_dw_rockchip_irq_rx_disable,
    .irq_tx_complete = uart_dw_rockchip_irq_tx_complete,
    .irq_rx_ready = uart_dw_rockchip_irq_rx_ready,
    .irq_err_enable = uart_dw_rockchip_irq_err_enable,
    .irq_err_disable = uart_dw_rockchip_irq_err_disable,
    .irq_is_pending = uart_dw_rockchip_irq_is_pending,
    .irq_update = uart_dw_rockchip_irq_update,
    .irq_callback_set = uart_dw_rockchip_irq_callback_set,
#endif
};

static int uart_dw_rockchip_init(const struct device *dev)
{
    const struct uart_dw_rockchip_config *config = dev->config;
    struct uart_dw_rockchip_data *data = dev->data;
    int ret;

    /* Apply pinctrl configuration */
    if (config->pcfg) {
        ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret < 0) {
            LOG_ERR("Failed to apply pinctrl state: %d", ret);
            return ret;
        }
    }

    /* Enable clocks */
    if (config->clock_dev) {
        ret = clock_control_on(config->clock_dev, config->clock_subsys);
        if (ret < 0) {
            LOG_ERR("Failed to enable clock: %d", ret);
            return ret;
        }
    }

    /* Reset the UART */
    if (reset_is_supported(&config->reset_spec)) {
        ret = reset_line_assert_dt(&config->reset_spec);
        if (ret < 0) {
            LOG_ERR("Failed to assert reset: %d", ret);
            return ret;
        }

        k_busy_wait(1);

        ret = reset_line_deassert_dt(&config->reset_spec);
        if (ret < 0) {
            LOG_ERR("Failed to deassert reset: %d", ret);
            return ret;
        }
    }

    /* Disable interrupts */
    uart_dw_rockchip_write(dev, UART_IER_DLH, 0);

    /* Set line control register */
    uart_dw_rockchip_write(dev, UART_LCR, config->line_ctrl);

    /* Configure FIFO */
    uart_dw_rockchip_write(dev, UART_IIR_FCR, 
                          UART_FCR_FIFOE | UART_FCR_RFIFOR | UART_FCR_TFIFOR);

    /* Set default baud rate */
    uart_dw_rockchip_set_baudrate(dev, 115200);
    data->baud_rate = 115200;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    /* Configure interrupt */
    if (config->irq_config_func) {
        config->irq_config_func(dev);
    }
#endif

    LOG_INF("UART initialized at 0x%08x", (uint32_t)config->reg_base);

    return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_DW_ROCKCHIP_IRQ_CONFIG_FUNC(n)                                   \
    static void uart_dw_rockchip_irq_config_func_##n(const struct device *dev)\
    {                                                                          \
        IRQ_CONNECT(DT_INST_IRQN(n),                                         \
                   DT_INST_IRQ(n, priority),                                  \
                   uart_dw_rockchip_isr,                                      \
                   DEVICE_DT_INST_GET(n),                                     \
                   0);                                                        \
        irq_enable(DT_INST_IRQN(n));                                         \
    }
#else
#define UART_DW_ROCKCHIP_IRQ_CONFIG_FUNC(n)
#endif

#define UART_DW_ROCKCHIP_INIT(n)                                             \
    UART_DW_ROCKCHIP_IRQ_CONFIG_FUNC(n)                                     \
                                                                             \
    PINCTRL_DT_INST_DEFINE(n);                                              \
                                                                             \
    static const struct uart_dw_rockchip_config uart_dw_rockchip_config_##n = { \
        .reg_base = DT_INST_REG_ADDR(n),                                    \
        .sys_clk_freq = DT_INST_PROP_OR(n, clock_frequency, 24000000),      \
        .clock_dev = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR_BY_IDX(n, 0)), \
        .clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id), \
        .reset_spec = RESET_DT_SPEC_INST_GET_OR(n, {0}),                    \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                          \
        .line_ctrl = UART_LCR_WLS_8,                                        \
        .fifo_size = DT_INST_PROP_OR(n, fifo_size, UART_FIFO_SIZE),        \
        IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                            \
                  (.irq_config_func = uart_dw_rockchip_irq_config_func_##n,))\
    };                                                                       \
                                                                             \
    static struct uart_dw_rockchip_data uart_dw_rockchip_data_##n = {       \
        .lock = {},                                                          \
    };                                                                       \
                                                                             \
    DEVICE_DT_INST_DEFINE(n,                                                \
                         uart_dw_rockchip_init,                             \
                         NULL,                                               \
                         &uart_dw_rockchip_data_##n,                        \
                         &uart_dw_rockchip_config_##n,                      \
                         PRE_KERNEL_1,                                       \
                         CONFIG_UART_INIT_PRIORITY,                          \
                         &uart_dw_rockchip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_DW_ROCKCHIP_INIT)