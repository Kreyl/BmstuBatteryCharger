/*
 * cmd_uart.h
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#ifndef UART_G070_H__
#define UART_G070_H__

#include "kl_libG070.h"
#include <cstring>
#include "shell.h"
#include "board.h"

extern "C"
void DmaUartTxIrq(void *p, uint32_t flags);

struct UartParams_t {
    uint32_t Baudrate;
    USART_TypeDef* Uart;
    GPIO_TypeDef *PGpioTx;
    uint16_t PinTx;
    GPIO_TypeDef *PGpioRx;
    uint16_t PinRx;
    // DMA
    uint32_t DmaChnlTx, DmaChnlRx;
    uint32_t DmaModeTx, DmaModeRx;
    uint32_t DmamuxReqIDTx, DmamuxReqIDRx;
    uartClk_t ClkSrc;
    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart,
            GPIO_TypeDef *APGpioTx, uint16_t APinTx,
            GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            uint32_t ADmaChnlTx, uint32_t ADmaChnlRx,
            uint32_t ADmaModeTx, uint32_t ADmaModeRx,
            uint32_t ADmamuxReqIDTx, uint32_t ADmamuxReqIDRx,
            uartClk_t AClkSrc
    ) : Baudrate(ABaudrate), Uart(AUart),
            PGpioTx(APGpioTx), PinTx(APinTx), PGpioRx(APGpioRx), PinRx(APinRx),
            DmaChnlTx(ADmaChnlTx), DmaChnlRx(ADmaChnlRx),
            DmaModeTx(ADmaModeTx), DmaModeRx(ADmaModeRx),
            DmamuxReqIDTx(ADmamuxReqIDTx), DmamuxReqIDRx(ADmamuxReqIDRx),
            ClkSrc(AClkSrc)
    {}
};

#define UART_CMD_BUF_SZ     54 // payload bytes
#define UART_RX_POLLING_MS  99

// ==== Base class ====
void UartDmaTxIrqHandler(void *p, uint32_t flags);

class BaseUart_t {
protected:
    const UartParams_t *Params;
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool ITxDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
    int32_t OldWIndx, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
protected:
    DMA_t DmaTx, DmaRx;
    uint8_t IPutByte(uint8_t b);
    void IStartTransmissionIfNotYet();
    // ==== Constructor ====
    BaseUart_t(const UartParams_t *APParams) : Params(APParams)
    , PRead(TXBuf), PWrite(TXBuf), ITxDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0)
    , OldWIndx(0), RIndx(0),
    DmaTx(Params->DmaChnlTx, Params->DmamuxReqIDTx, UartDmaTxIrqHandler, this),
    DmaRx(Params->DmaChnlRx, Params->DmamuxReqIDRx) {}
    uint8_t GetByte(uint8_t *b);
public:
    void Init();
    void Shutdown();
    void OnClkChange();
    uint8_t PutByteNow(uint8_t b);
    // Enable/Disable
    void Enable()    { Params->Uart->CR1 |= USART_CR1_UE; }
    void EnableTx()  { Params->Uart->CR1 |= USART_CR1_TE; }
    void DisableTx() { Params->Uart->CR1 &= ~USART_CR1_TE; }
    void EnableRx()  { Params->Uart->CR1 |= USART_CR1_RE; }
    void DisableRx() { Params->Uart->CR1 &= ~USART_CR1_RE; }
    void FlushTx() { while(!ITxDmaIsIdle); }  // wait DMA
    void EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback);
    // Inner use
    void IRQDmaTxHandler();
    void IRQUartHandler(uint32_t flags) { }
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    uint8_t IPutChar(char c) override { return IPutByte(c);  };
    void IStartTransmissionIfNotYet() override { BaseUart_t::IStartTransmissionIfNotYet(); }
public:
    CmdUart_t(const UartParams_t *APParams) : BaseUart_t(APParams) {}
    void Init();
    void Print(const char *format, ...) override {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    uint8_t GetRcvdCmd();
    // Inner use
    void IRQUartHandler(uint32_t flags);
};

#endif // UART_G070_H__
