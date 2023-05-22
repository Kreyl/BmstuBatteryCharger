#ifndef BOARD_H__
#define BOARD_H__

// ==== General ====
#define BOARD_NAME          "Test"
#define APP_NAME            "BatteryCharger"

#define LSE_FREQ_HZ     32768   // Left it here even if not used

#ifndef TRUE
#define TRUE            1
#endif

#if 1 // ========================== GPIO =======================================
// UART
#define UART_GPIO       GPIOA
//#define UART_TX_PIN     9
//#define UART_RX_PIN     10
#define UART_TX_PIN     2
#define UART_RX_PIN     3

// Debug pin
#define DBG_PIN         GPIOB, 0
#define DBG_HI()        PinSetHi(DBG_PIN)
#define DBG_LO()        PinSetLo(DBG_PIN)
#define DBG_TOGGLE()    PinToggle(DBG_PIN)

#endif // GPIO

#if 0 // =========================== ADC =======================================
#define ADC_ENABLED     TRUE
#define ADC_CHNL_CNT    2
#define ADC_DMA_ENABLE  FALSE

#endif

#if 1 // =========================== Timers ====================================
#define TIME_TIMER      TIM7
//#define ADC_TIMER       TIM6

#endif

#if 1 // =========================== DMA =======================================
// ==== Uart ====
#define UART_DMA_TX_CHNL    7
//#define UART_DMA_TX_REQID   51 // p223 of refman
#define UART_DMA_TX_REQID   53 // p223 of refman
#define UART_DMA_RX_CHNL    6
//#define UART_DMA_RX_REQID   50 // p223 of refman
#define UART_DMA_RX_REQID   52 // p223 of refman

#define UART_DMA_TX_MODE (  DMA_PRIORITY_LOW | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |       /* Memory pointer increase */ \
                            DMA_DIR_MEM2PER |    /* Direction is memory to peripheral */ \
                            DMA_TCIE         /* Enable Transmission Complete IRQ */)

#define UART_DMA_RX_MODE (  DMA_PRIORITY_MEDIUM | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |       /* Memory pointer increase */ \
                            DMA_DIR_PER2MEM |    /* Direction is peripheral to memory */ \
                            DMA_CIRC         /* Circular buffer enable */)

// ==== Ctrl SPI ====
#define CTRL_DMA_TX_CHNL    3
#define CTRL_DMA_TX_REQID   19 // SPI2 TX, p223 of refman
#define CTRL_DMA_RX_CHNL    2
#define CTRL_DMA_RX_REQID   18 // SPI2 RX, p223 of refman

#define CTRL_DMA_TX_MODE (  DMA_PRIORITY_LOW | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |     \
                            DMA_DIR_MEM2PER )

#define CTRL_DMA_RX_MODE (  DMA_PRIORITY_MEDIUM | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |     \
                            DMA_DIR_PER2MEM )

// ==== I2C1 ====
#define I2C1_DMA_TX_CHNL    5
#define I2C1_DMA_TX_REQID   11
#define I2C1_DMA_RX_CHNL    4
#define I2C1_DMA_RX_REQID   10

// ==== ADC ====
#define ADC_DMA_CHNL        1
#define ADC_DMA_REQID       5 // ADC, p223 of refman

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN     FALSE
#define UART_TXBUF_SZ       256
#define UART_RXBUF_SZ       128
#define CMD_BUF_SZ          99
#define UART_CHAR_TO_IRQ    '\n'
#define DBG_UART            USART2

#define CMD_UART_PARAMS \
    DBG_UART, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX_CHNL, UART_DMA_RX_CHNL, \
    UART_DMA_TX_MODE, UART_DMA_RX_MODE,\
    UART_DMA_TX_REQID, UART_DMA_RX_REQID, \
    uartclkHSI

#endif

#endif // BOARD_H__
