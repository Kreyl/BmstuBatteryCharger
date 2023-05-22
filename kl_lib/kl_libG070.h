/*
 * kl_libG070.h
 *
 *  Created on: 17 ���. 2020 �.
 *      Author: layst
 */

#ifndef KLLIBG070_H__
#define KLLIBG070_H__

#include "board.h"
#include <stdint.h>
#include "stm32g0xx.h"
#include <stdlib.h>

#if 1 // ============================= General =================================
typedef void (*ftVoidVoid)(void);
typedef void (*ftVoidPVoid)(void*p);
typedef void (*ftVoidPVoidW32)(void*p, uint32_t W32);

void Printf(const char *format, ...);

#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

// IRQ priorities
#define IRQ_PRIO_LOW            15  // Minimum
#define IRQ_PRIO_MEDIUM         9
#define IRQ_PRIO_HIGH           7
#define IRQ_PRIO_VERYHIGH       4   // Higher than systick

// Return values
#define retvOk              0
#define retvFail            1
#define retvTimeout         2
#define retvBusy            3
#define retvInProgress      4
#define retvCmdError        5
#define retvCmdUnknown      6
#define retvBadValue        7
#define retvNew             8
#define retvSame            9
#define retvLast            10
#define retvEmpty           11
#define retvOverflow        12
#define retvNotANumber      13
#define retvWriteProtect    14
#define retvWriteError      15
#define retvEndOfFile       16
#define retvNotFound        17
#define retvBadState        18
#define retvDisconnected    19
#define retvCollision       20
#define retvCRCError        21
#define retvNACK            22
#define retvNoAnswer        23
#define retvOutOfMemory     24
#define retvNotAuthorised   25
#define retvNoChanges       26

enum RiseFall_t {risefallRising, risefallFalling, risefallNone, risefallBoth};

// ==== Build time ====
// Define symbol BUILD_TIME in main.cpp options with value ${current_date}.
// Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
#define STRINGIFY(x)    # x
#define XSTRINGIFY(x)   STRINGIFY(x)

// ==== Math ====
#define MIN_(a, b)   ( ((a)<(b))? (a) : (b) )
#define MAX_(a, b)   ( ((a)>(b))? (a) : (b) )
#define ABS(a)      ( ((a) < 0)? -(a) : (a) )

#define REBOOT()    SCB->AIRCR = 0x05FA0004
#endif

#if 1 // ========================== Random =====================================
namespace Random {
//uint32_t last = 1;
// Generate pseudo-random value
static inline long int Generate(long int LowInclusive, long int HighInclusive) {
    uint32_t last = random();
    return (last % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
}
// Seed pseudo-random generator with new seed
static inline void Seed(unsigned int Seed) { srandom(Seed); }
} // namespace
#endif

#if 1 // ============================== RCC ====================================
#define HSI_FREQ_HZ         16000000
#define LSI_FREQ_HZ         32000
enum uartClk_t {uartclkPCLK = 0, uartclkSYSCLK = 1, uartclkHSI = 2, uartclkLSE = 3 };

extern uint32_t AHBFreqHz, APBFreqHz;

static inline void EnableAHBClk(uint32_t AhbMsk) {
    RCC->AHBENR |= AhbMsk;
    // Delay after an RCC peripheral clock enabling
    volatile uint32_t tmpreg = RCC->AHBENR & AhbMsk;
    (void)tmpreg;
}

static inline void EnableAPB1Clk(uint32_t Apb1Msk) {
    RCC->APBENR1 |= Apb1Msk;
    // Delay after an RCC peripheral clock enabling
    volatile uint32_t tmpreg = RCC->APBENR1 & Apb1Msk;
    (void)tmpreg;
}

static inline void EnableAPB2Clk(uint32_t Apb2Msk) {
    RCC->APBENR2 |= Apb2Msk;
    // Delay after an RCC peripheral clock enabling
    volatile uint32_t tmpreg = RCC->APBENR2 & Apb2Msk;
    (void)tmpreg;
}

static inline void EnableLSI() {
    RCC->CSR |= RCC_CSR_LSION;
    while(!(RCC->CSR & RCC_CSR_LSIRDY));
}

static inline uint32_t GetSysClkHz() { // TODO
    return 64000000;
}
#endif

#if 1 // ===================== Simple pin manipulations ========================
enum PinPullUpDown_t {
    pudNone = 0b00,
    pudPullUp = 0b01,
    pudPullDown = 0b10
};

enum PinSpeed_t {
    psVeryLow = 0b00,
    psLow = 0b01,
    psHigh = 0b10,
    psVeryHigh = 0b11
};
#define PIN_SPEED_DEFAULT   psLow

enum PinOutMode_t {omPushPull = 0, omOpenDrain = 1};

enum AlterFunc_t { AF0=0, AF1=1, AF2=2, AF3=3, AF4=4, AF5=5, AF6=6, AF7=7 };

__attribute__((__always_inline__))
static inline void PinSetHi(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BSRR = 1 << APin; }
__attribute__((__always_inline__))
static inline void PinSetLo(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BRR = 1 << APin;  }
__attribute__((__always_inline__))
static inline void PinToggle(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->ODR ^= 1 << APin; }
// Check input
__attribute__((__always_inline__))
static inline bool PinIsHi(GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->IDR & (1 << APin); }
__attribute__((__always_inline__))
static inline bool PinIsHi(const GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->IDR & (1 << APin); }
__attribute__((__always_inline__))
static inline bool PinIsLo(GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->IDR & (1 << APin)); }
__attribute__((__always_inline__))
static inline bool PinIsLo(const GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->IDR & (1 << APin)); }

static void PinClockEnable(const GPIO_TypeDef *PGpioPort) {
    volatile uint32_t tmpreg;
    if(PGpioPort == GPIOA) {
        RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
        tmpreg = RCC->IOPENR & RCC_IOPENR_GPIOAEN; // Delay after an RCC peripheral clock enabling
    }
    else if(PGpioPort == GPIOB) {
        RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
        tmpreg = RCC->IOPENR & RCC_IOPENR_GPIOBEN; // Delay after an RCC peripheral clock enabling
    }
    else if(PGpioPort == GPIOC) {
        RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
        tmpreg = RCC->IOPENR & RCC_IOPENR_GPIOCEN; // Delay after an RCC peripheral clock enabling
    }
    else if(PGpioPort == GPIOD) {
        RCC->IOPENR |= RCC_IOPENR_GPIODEN;
        tmpreg = RCC->IOPENR & RCC_IOPENR_GPIODEN; // Delay after an RCC peripheral clock enabling
    }
    else if(PGpioPort == GPIOF) {
        RCC->IOPENR |= RCC_IOPENR_GPIOFEN;
        tmpreg = RCC->IOPENR & RCC_IOPENR_GPIOFEN; // Delay after an RCC peripheral clock enabling
    }
    (void)tmpreg;
}

static inline void PinSetupOut(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT
        ) {
    // Clock
    PinClockEnable(PGpioPort);
    uint32_t Offset = APinNumber*2;
    // Setup mode
    PGpioPort->MODER &= ~(0b11 << Offset);  // clear previous bits
    PGpioPort->MODER |=   0b01 << Offset;   // Set new bits
    // Setup output type
    PGpioPort->OTYPER &= ~(1<<APinNumber);
    PGpioPort->OTYPER |= (uint32_t)PinOutMode << APinNumber;
    // Setup Pull-Up or Pull-Down
    PGpioPort->PUPDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->PUPDR |= (uint32_t)pudNone << Offset;
    // Setup speed
    PGpioPort->OSPEEDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->OSPEEDR |= (uint32_t)ASpeed << Offset;
}

static inline void PinSetupInput(
        GPIO_TypeDef *PGpio,
        const uint16_t PinN,
        const PinPullUpDown_t PullUpDown,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT) {
    uint32_t Offset = PinN*2;
    // Clock
    PinClockEnable(PGpio);
    // Setup mode
    PGpio->MODER &= ~(0b11 << Offset); // clear previous bits
    // Setup Pull-Up or Pull-Down
    PGpio->PUPDR &= ~(0b11 << Offset); // clear previous bits
    PGpio->PUPDR |= (uint32_t)PullUpDown << Offset;
}

static inline void PinSetupAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    // Clock
    PinClockEnable(PGpioPort);
    // Setup mode
    PGpioPort->MODER |= 0b11 << (APinNumber*2);  // Set new bits
}

static inline void PinSetupAlterFunc(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinPullUpDown_t APullUpDown,
        const AlterFunc_t AAlterFunc,
        const PinSpeed_t ASpeed = PIN_SPEED_DEFAULT) {
    // Clock
    PinClockEnable(PGpioPort);
    uint32_t Offset = APinNumber*2;
    // Setup mode
    PGpioPort->MODER &= ~(0b11 << Offset);  // clear previous bits
    PGpioPort->MODER |= 0b10 << Offset;     // Set new bits (AF mode)
    // Setup output type
    if(PinOutMode == omPushPull) PGpioPort->OTYPER &= ~(1<<APinNumber);
    else PGpioPort->OTYPER |= 1 << APinNumber;  // Open Drain
    // Setup Pull-Up or Pull-Down
    PGpioPort->PUPDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->PUPDR |= (uint32_t)APullUpDown << Offset;
    // Setup speed
    PGpioPort->OSPEEDR &= ~(0b11 << Offset); // clear previous bits
    PGpioPort->OSPEEDR |= (uint32_t)ASpeed << Offset;
    // Setup Alternate Function
    uint32_t n = (APinNumber <= 7)? 0 : 1;      // 0 if 0...7, 1 if 8..15
    Offset = 4 * ((APinNumber <= 7)? APinNumber : APinNumber - 8);
    PGpioPort->AFR[n] &= ~(0b1111 << Offset);
    PGpioPort->AFR[n] |= (uint32_t)AAlterFunc << Offset;
}
#endif

#if 1 // =========================== External IRQ ==============================
// Pin to IRQ channel
#define PIN2IRQ_CHNL(Pin)   (((Pin) > 3)? EXTI4_15_IRQn : (((Pin) > 1)? EXTI2_3_IRQn : EXTI0_1_IRQn))

// IRQ handlers
extern "C" {
extern ftVoidVoid ExtiIrqHandler_0_1, ExtiIrqHandler_2_3, ExtiIrqHandler_4_15;
}

class PinIrq_t {
public:
    GPIO_TypeDef *PGpio;
    uint16_t PinN;
    PinPullUpDown_t PullUpDown;
    PinIrq_t(GPIO_TypeDef *APGpio, uint16_t APinN, PinPullUpDown_t APullUpDown, ftVoidVoid PIrqHandler) :
        PGpio(APGpio), PinN(APinN), PullUpDown(APullUpDown) {
        if(APinN == 0 or APinN == 1) ExtiIrqHandler_0_1 = PIrqHandler;
        else if(APinN == 2 or APinN == 3) ExtiIrqHandler_2_3 = PIrqHandler;
        else ExtiIrqHandler_4_15 = PIrqHandler;
    }

    bool IsHi() const { return PinIsHi(PGpio, PinN); }

    void SetTriggerType(RiseFall_t ATriggerType) const {
        uint32_t IrqMsk = 1 << PinN;
        switch(ATriggerType) {
            case risefallRising:
                EXTI->RTSR1 |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR1 &= ~IrqMsk;  // Falling trigger disabled
                break;
            case risefallFalling:
                EXTI->RTSR1 &= ~IrqMsk;  // Rising trigger disabled
                EXTI->FTSR1 |=  IrqMsk;  // Falling trigger enabled
                break;
            case risefallBoth:
                EXTI->RTSR1 |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTSR1 |=  IrqMsk;  // Falling trigger enabled
                break;
            default: break;
        } // switch
    }

    void Init(RiseFall_t ATriggerType) const {
        // Init pin as input
        PinSetupInput(PGpio, PinN, PullUpDown);
        // Connect EXTI line to the pin of the port
        uint8_t Indx   = PinN / 4;               // Indx of EXTICR register
        uint8_t Offset = (PinN & 0x03) * 8;      // Offset in EXTICR register
        EXTI->EXTICR[Indx] &= ~((uint32_t)0b1111 << Offset);  // Clear port-related bits
        // GPIOA requires all zeroes => nothing to do in this case
        if     (PGpio == GPIOB) EXTI->EXTICR[Indx] |= 1UL << Offset;
        else if(PGpio == GPIOC) EXTI->EXTICR[Indx] |= 2UL << Offset;
        else if(PGpio == GPIOD) EXTI->EXTICR[Indx] |= 3UL << Offset;
        else if(PGpio == GPIOF) EXTI->EXTICR[Indx] |= 5UL << Offset;
        // Configure EXTI line
        uint32_t IrqMsk = 1 << PinN;
        EXTI->IMR1  |=  IrqMsk;      // Interrupt mode enabled
        EXTI->EMR1  &= ~IrqMsk;      // Event mode disabled
        SetTriggerType(ATriggerType);
        EXTI->RPR1 = IrqMsk;      // Clean rising irq flag
        EXTI->FPR1 = IrqMsk;      // Clean falling irq flag
    }
    void EnableIrq(const uint32_t Priority) const {
        NVIC_SetPriority(PIN2IRQ_CHNL(PinN), Priority);
        NVIC_EnableIRQ(PIN2IRQ_CHNL(PinN));
    }
    void DisableIrq()   const { NVIC_DisableIRQ(PIN2IRQ_CHNL(PinN)); }
    void CleanIrqFlag() const {
        EXTI->RPR1 = (1 << PinN);
        EXTI->FPR1 = (1 << PinN);
        NVIC_ClearPendingIRQ(PIN2IRQ_CHNL(PinN));
    }
    bool IsIrqPending() const { return (EXTI->RPR1 & (1 << PinN)) or (EXTI->FPR1 & (1 << PinN)); }
    void GenerateIrq()  const { EXTI->SWIER1 = (1 << PinN); }
};
#endif // EXTI

#if 1 // ============================== SPI ====================================
enum CPHA_t {cphaFirstEdge, cphaSecondEdge};
enum CPOL_t {cpolIdleLow, cpolIdleHigh};
enum SpiClkDivider_t {
    sclkDiv2   = 0b000,
    sclkDiv4   = 0b001,
    sclkDiv8   = 0b010,
    sclkDiv16  = 0b011,
    sclkDiv32  = 0b100,
    sclkDiv64  = 0b101,
    sclkDiv128 = 0b110,
    sclkDiv256 = 0b111,
};

enum BitOrder_t {boMSB, boLSB};
enum BitNumber_t {bitn8, bitn16, bitn32};

class Spi_t {
public:
    SPI_TypeDef *PSpi;
    Spi_t(SPI_TypeDef *ASpi) : PSpi(ASpi) {}
    // Example: boMSB, cpolIdleLow, cphaFirstEdge, sbFdiv2, bitn8
    void Setup(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA, int32_t Bitrate_Hz, BitNumber_t BitNumber) const;
    void SetupSlave(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA, BitNumber_t BitNumber) const;
    void Enable ()       const { PSpi->CR1 |=  SPI_CR1_SPE; }
    void Disable()       const { PSpi->CR1 &= ~SPI_CR1_SPE; }
    void Reset() const;

    // CRC
    void RestartCrc() {
        PSpi->CR1 &= ~SPI_CR1_CRCEN; // Clear CRCEN bit
        PSpi->CR1 |= SPI_CR1_CRCEN | SPI_CR1_CRCL;
        PSpi->SR &= ~SPI_SR_CRCERR;
        PSpi->CRCPR = 4129; // Polynom
    }
    uint16_t GetRxCrc() { return PSpi->RXCRCR; }
    bool IsRxCrcOk() { return !(PSpi->SR & SPI_SR_CRCERR); }

    // DMA
    void EnableTxDma()   const { PSpi->CR2 |=  SPI_CR2_TXDMAEN; }
    void DisableTxDma()  const { PSpi->CR2 &= ~SPI_CR2_TXDMAEN; }
    void EnableRxDma()   const { PSpi->CR2 |=  SPI_CR2_RXDMAEN; }
    void DisableRxDma()  const { PSpi->CR2 &= ~SPI_CR2_RXDMAEN; }

    // IRQ
    void EnableRxIrq()  const { PSpi->CR2 |=  SPI_CR2_RXNEIE; }
    void DisableRxIrq() const { PSpi->CR2 &= ~SPI_CR2_RXNEIE; }
    void EnableIrq(const uint32_t Priority) const {
//        if(PSpi == SPI1) nvicEnableVector(SPI1_IRQn, Priority);
//        else if(PSpi == SPI2) nvicEnableVector(SPI2_IRQn, Priority);
    }
    void DisableIrq() const {
//        if(PSpi == SPI1) nvicDisableVector(SPI1_IRQn);
//        else if(PSpi == SPI2) nvicDisableVector(SPI2_IRQn);
    }
//    void SetupRxIrqCallback(ftVoidVoid AIrqHandler) const;

    // Rx/Tx
    void SetRxOnly()     const { PSpi->CR1 |=  SPI_CR1_RXONLY; }
    void SetFullDuplex() const { PSpi->CR1 &= ~SPI_CR1_RXONLY; }
    // Flags
#if defined STM32F072xB
    void WaitFTLVLZero() const { while(PSpi->SR & SPI_SR_FTLVL); }
#endif
    void WaitBsyHi2Lo()  const { while(PSpi->SR & SPI_SR_BSY); }
    void WaitTxEHi()     const { while(!(PSpi->SR & SPI_SR_TXE)); }
    void ClearRxBuf()    const { while(PSpi->SR & SPI_SR_RXNE) (void)PSpi->DR; }
    void ClearOVR()      const { (void)PSpi->DR; (void)PSpi->SR; (void)PSpi->DR; }

    // Read/Write
    uint8_t ReadByte() const { return PSpi->DR; }
    void WriteByte(uint8_t AByte) const { *((volatile uint8_t*)&PSpi->DR) = AByte; }
    void WriteWord(uint16_t AWord) const { PSpi->DR = AWord; }

    uint8_t ReadWriteByte(uint8_t AByte) const {
        *((volatile uint8_t*)&PSpi->DR) = AByte;
        while(!(PSpi->SR & SPI_SR_RXNE));  // Wait for SPI transmission to complete
        return *((volatile uint8_t*)&PSpi->DR);
    }
    uint16_t ReadWriteWord(uint16_t Word) const {
        PSpi->DR = Word;
        while(!(PSpi->SR & SPI_SR_RXNE));
        return PSpi->DR;
    }
};

class I2S_t {
public:
    SPI_TypeDef *PSpi;
    I2S_t(SPI_TypeDef *ASpi) : PSpi(ASpi) {}
    void SetupI2SMasterTx16Bit() const;
    void SetPrescaler(uint16_t Psc) const { PSpi->I2SPR = Psc; }
    void Enable()  const { PSpi->I2SCFGR |=  SPI_I2SCFGR_I2SE; }
    void Disable() const { PSpi->I2SCFGR &= ~SPI_I2SCFGR_I2SE; }
    void Reset() const;
    // DMA
    void EnableTxDma()   const { PSpi->CR2 |=  SPI_CR2_TXDMAEN; }
    void DisableTxDma()  const { PSpi->CR2 &= ~SPI_CR2_TXDMAEN; }
};

#endif

#if 1 // =========================== HW Timers =================================
enum TmrTrigInput_t {
    tiITR0=  0x000000,
    tiITR1=  0x000010,
    tiITR2=  0x000020,
    tiITR3=  0x000030,
    tiTIED=  0x000040,
    tiTI1FP1=0x000050,
    tiTI2FP2=0x000060,
    tiETRF=  0x000070,
    tiITR4=  0x100000,
    tiITR5=  0x100001,
    tiITR6=  0x100002,
    tiITR7=  0x100003,
    tiITR8=  0x100004,
};
enum TmrMasterMode_t {mmReset=0x00, mmEnable=0x10, mmUpdate=0x20, mmComparePulse=0x30, mmCompare1=0x40, mmCompare2=0x50, mmCompare3=0x60, mmCompare4=0x70};
enum TmrSlaveMode_t {smDisable=0, smEncoder1=1, smEncoder2=2, smEncoder3=3, smReset=4, smGated=5, smTrigger=6, smExternal=7};
enum ExtTrigPol_t {etpInverted=0x8000, etpNotInverted=0x0000};
enum ExtTrigPsc_t {etpOff=0x0000, etpDiv2=0x1000, etpDiv4=0x2000, etpDiv8=0x30000};

enum Inverted_t {invNotInverted, invInverted};

#define TMR_PCCR(PTimer, AChannel)  ((uint32_t*)(&PTimer->CCR1 + AChannel-1))
#define TMR_ENABLE(PTimer)          PTimer->CR1 |=  TIM_CR1_CEN;
#define TMR_DISABLE(PTimer)         PTimer->CR1 &= ~TIM_CR1_CEN;
#define TMR_GENERATE_UPD(PTimer)    PTimer->EGR = TIM_EGR_UG;

class Timer_t {
protected:
    TIM_TypeDef* ITmr;
public:
    Timer_t(TIM_TypeDef *APTimer) : ITmr(APTimer) {}
    void Init() const;
    void Deinit() const;
    void Enable() const { TMR_ENABLE(ITmr); }
    void Disable() const { TMR_DISABLE(ITmr); }
    void Reset() const;
    void SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const;
    void SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const;
    void SetUpdateFrequencyChangingBoth(uint32_t FreqHz) const;
    void SetTopValue(uint32_t Value) const { ITmr->ARR = Value; }
    uint32_t GetTopValue() const { return ITmr->ARR; }
    void EnableArrBuffering()  const { ITmr->CR1 |=  TIM_CR1_ARPE; }
    void DisableArrBuffering() const { ITmr->CR1 &= ~TIM_CR1_ARPE; }
    void SetupPrescaler(uint32_t PrescaledFreqHz) const;
    void SetCounter(uint32_t Value) const { ITmr->CNT = Value; }
    uint32_t GetCounter() const { return ITmr->CNT; }

    // Compare
    void SetCCR1(uint32_t AValue) const { ITmr->CCR1 = AValue; }
    void SetCCR2(uint32_t AValue) const { ITmr->CCR2 = AValue; }
    void SetCCR3(uint32_t AValue) const { ITmr->CCR3 = AValue; }
    void SetCCR4(uint32_t AValue) const { ITmr->CCR4 = AValue; }

    // Inputs
    enum InputPresacaler_t{pscDiv1=0UL, pscDiv2=01UL, pscDiv4=2UL, pscDiv8=3UL};
    void SetupInput1(uint32_t Mode, InputPresacaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CCMR1 = (ITmr->CCMR1 & 0xFF00) | ((uint32_t)Psc << 2)  | (Mode << 0);
        uint16_t bits = (Rsfll == risefallRising)? 0b0000U : (Rsfll == risefallFalling)? 0b0010U : 0b1010;
        ITmr->CCER = (ITmr->CCER & ~(0xAU << 0)) | (bits << 0);
    }
    void SetupInput2(uint32_t Mode, InputPresacaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CCMR1 = (ITmr->CCMR1 & 0x00FF) | ((uint32_t)Psc << 10) | (Mode << 8);
        uint16_t bits = (Rsfll == risefallRising)? 0b0000U : (Rsfll == risefallFalling)? 0b0010U : 0b1010;
        ITmr->CCER = (ITmr->CCER & ~(0xAU << 4)) | (bits << 4);
    }
    void SetupInput3(uint32_t Mode, InputPresacaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CCMR2 = (ITmr->CCMR2 & 0xFF00) | ((uint32_t)Psc << 2)  | (Mode << 0);
        uint16_t bits = (Rsfll == risefallRising)? 0b0000U : (Rsfll == risefallFalling)? 0b0010U : 0b1010;
        ITmr->CCER = (ITmr->CCER & ~(0xAU << 8)) | (bits << 8);
    }
    void SetupInput4(uint32_t Mode, InputPresacaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CCMR2 = (ITmr->CCMR2 & 0x00FF) | ((uint32_t)Psc << 10) | (Mode << 8);
        uint16_t bits = (Rsfll == risefallRising)? 0b0000U : (Rsfll == risefallFalling)? 0b0010U : 0b1010;
        ITmr->CCER = (ITmr->CCER & ~(0xAU << 12)) | (bits << 12);
    }

    // Outputs
    void SetupOutput1(uint32_t Mode) const { ITmr->CCMR1 = (ITmr->CCMR1 & 0xFFFEFF00) | ((Mode & 8UL) << 13) | ((Mode & 7UL) << 4); }
    void SetupOutput2(uint32_t Mode) const { ITmr->CCMR1 = (ITmr->CCMR1 & 0xFEFF00FF) | ((Mode & 8UL) << 21) | ((Mode & 7UL) << 12); }
    void SetupOutput3(uint32_t Mode) const { ITmr->CCMR2 = (ITmr->CCMR2 & 0xFFFEFF00) | ((Mode & 8UL) << 13) | ((Mode & 7UL) << 4); }
    void SetupOutput4(uint32_t Mode) const { ITmr->CCMR2 = (ITmr->CCMR2 & 0xFEFF00FF) | ((Mode & 8UL) << 21) | ((Mode & 7UL) << 12); }
    void EnableCCOutput1() const { ITmr->CCER |= TIM_CCER_CC1E; }
    void EnableCCOutput2() const { ITmr->CCER |= TIM_CCER_CC2E; }
    void EnableCCOutput3() const { ITmr->CCER |= TIM_CCER_CC3E; }
    void EnableCCOutput4() const { ITmr->CCER |= TIM_CCER_CC4E; }

    // Master/Slave
    void SetTriggerInput(TmrTrigInput_t TrgInput) const {
        uint32_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_TS;   // Clear bits
        tmp |= (uint32_t)TrgInput;
        ITmr->SMCR = tmp;
    }
    void SetEtrPolarity(Inverted_t AInverted) {
        if(AInverted == invInverted) ITmr->SMCR |= TIM_SMCR_ETP;
        else ITmr->SMCR &= ~TIM_SMCR_ETP;
    }
    void SelectMasterMode(TmrMasterMode_t MasterMode) const {
        uint32_t tmp = ITmr->CR2;
        tmp &= ~TIM_CR2_MMS;
        tmp |= (uint32_t)MasterMode;
        ITmr->CR2 = tmp;
    }
    void SelectSlaveMode(TmrSlaveMode_t SlaveMode) const {
        uint32_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_SMS;
        tmp |= (uint32_t)SlaveMode;
        ITmr->SMCR = tmp;
    }

    // DMA, Irq, Evt
    void EnableDmaOnTrigger() const { ITmr->DIER |= TIM_DIER_TDE; }
    void EnableDMAOnCapture(uint8_t CaptureReq) const { ITmr->DIER |= (1 << (CaptureReq + 8)); }
    void GenerateUpdateEvt()  const { ITmr->EGR = TIM_EGR_UG; }
    // Enable
    void EnableIrq(IRQn_Type IrqChnl, uint32_t IrqPriority) const {
        NVIC_SetPriority(IrqChnl, IrqPriority);
        NVIC_EnableIRQ(IrqChnl);
    }
    void EnableIrqOnUpdate()  const { ITmr->DIER |= TIM_DIER_UIE; }
    void EnableIrqOnCompare1() const { ITmr->DIER |= TIM_DIER_CC1IE; }
    void EnableIrqOnCompare2() const { ITmr->DIER |= TIM_DIER_CC2IE; }
    void EnableIrqOnCompare3() const { ITmr->DIER |= TIM_DIER_CC3IE; }
    void EnableIrqOnCompare4() const { ITmr->DIER |= TIM_DIER_CC4IE; }
    // Clear
    void ClearUpdateIrqPendingBit()   const { ITmr->SR &= ~TIM_SR_UIF; }
    void ClearCompare1IrqPendingBit() const { ITmr->SR &= ~TIM_SR_CC1IF; }
    void ClearCompare2IrqPendingBit() const { ITmr->SR &= ~TIM_SR_CC2IF; }
    void ClearCompare3IrqPendingBit() const { ITmr->SR &= ~TIM_SR_CC3IF; }
    void ClearCompare4IrqPendingBit() const { ITmr->SR &= ~TIM_SR_CC4IF; }
    // Check
    bool IsUpdateIrqFired() const { return (ITmr->SR & TIM_SR_UIF); }
    bool IsCompare1IrqFired() const { return (ITmr->SR & TIM_SR_CC1IF); }
    bool IsCompare2IrqFired() const { return (ITmr->SR & TIM_SR_CC2IF); }
    bool IsCompare3IrqFired() const { return (ITmr->SR & TIM_SR_CC3IF); }
    bool IsCompare4IrqFired() const { return (ITmr->SR & TIM_SR_CC4IF); }
};
#endif

#if 1 // ============================== PWM ====================================
/* Example:
 * #define LED_R_PIN { GPIOB, 1, TIM3, 4, invInverted, omPushPull, 255 }
 * PinOutputPWM_t Led {LedPin};
*/
struct PwmSetup_t {
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    AlterFunc_t AF;
    TIM_TypeDef *PTimer;
    uint32_t TimerChnl;
    Inverted_t Inverted;
    PinOutMode_t OutputType;
    uint32_t TopValue;
    PwmSetup_t(GPIO_TypeDef *APGpio, uint16_t APin, AlterFunc_t AAF,
            TIM_TypeDef *APTimer, uint32_t ATimerChnl,
            Inverted_t AInverted, PinOutMode_t AOutputType,
            uint32_t ATopValue) : PGpio(APGpio), Pin(APin), AF(AAF),
                    PTimer(APTimer), TimerChnl(ATimerChnl),
                    Inverted(AInverted), OutputType(AOutputType),
                    TopValue(ATopValue) {}
};

class PinOutputPWM_t : private Timer_t {
private:
    const PwmSetup_t ISetup;
public:
    void Set(const uint16_t AValue) const { *TMR_PCCR(ITmr, ISetup.TimerChnl) = AValue; }    // CCR[N] = AValue
    uint32_t Get() const { return *TMR_PCCR(ITmr, ISetup.TimerChnl); }
    void Init() const;
    void Deinit() const { Timer_t::Deinit(); PinSetupAnalog(ISetup.PGpio, ISetup.Pin); }
    void SetFrequencyHz(uint32_t FreqHz) const { Timer_t::SetUpdateFrequencyChangingPrescaler(FreqHz); }
    PinOutputPWM_t(const PwmSetup_t &ASetup) : Timer_t(ASetup.PTimer), ISetup(ASetup) {}
    PinOutputPWM_t(GPIO_TypeDef *PGpio, uint16_t Pin, AlterFunc_t AAF,
            TIM_TypeDef *PTimer, uint32_t TimerChnl,
            Inverted_t Inverted, PinOutMode_t OutputType,
            uint32_t TopValue) : Timer_t(PTimer),
                    ISetup(PGpio, Pin, AAF, PTimer, TimerChnl, Inverted, OutputType, TopValue) {}
};
#endif

#if 1 // ========================= Time and Delay ==============================
class Time_t : private Timer_t {
public:
    Time_t(TIM_TypeDef *APTimer) : Timer_t(APTimer) {}
    void Init();
    uint32_t GetCurrent() { return GetCounter(); }
    void Wait(uint16_t Time_ms);
    uint32_t ElapsedSince(uint32_t Start);
};

// Place somewhere Time_t Time{TIME_TIMER};

static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }
#endif

#if 1 // ============================== DMA ====================================
// DMA
#define DMA_PRIORITY_LOW        (0b00UL << 12)
#define DMA_PRIORITY_MEDIUM     (0b01UL << 12)
#define DMA_PRIORITY_HIGH       (0b10UL << 12)
#define DMA_PRIORITY_VERYHIGH   (0b11UL << 12)

#define DMA_MSIZE_8_BIT         (0b00UL << 10)
#define DMA_MSIZE_16_BIT        (0b01UL << 10)
#define DMA_MSIZE_32_BIT        (0b10UL << 10)

#define DMA_PSIZE_8_BIT         (0b00UL << 8)
#define DMA_PSIZE_16_BIT        (0b01UL << 8)
#define DMA_PSIZE_32_BIT        (0b10UL << 8)

#define DMA_MEM_INC             (1UL << 7)
#define DMA_PER_INC             (1UL << 6)
#define DMA_CIRC                (1UL << 5)

#define DMA_DIR_MEM2PER         (1UL << 4)
#define DMA_DIR_PER2MEM         (0UL << 4)

#define DMA_TCIE                (1UL << 1)

#define DMA_CHNL_CNT            7

struct Dmamux_t {
    volatile uint32_t CCR[7]; // 0x00...0x18
    volatile uint32_t Reserved1[24]; // 0x1C...0x7C
    volatile uint32_t CSR; // 0x80
    volatile uint32_t CFR; // 0x84
    volatile uint32_t Reserved2[29];
    volatile uint32_t RGCR[4]; // 0x100...0x10C
    volatile uint32_t Reserved3[11];
    volatile uint32_t RGSR; // 0x140
    volatile uint32_t RGCFR; // 0x144
};

#define DMAMUX  ((Dmamux_t*)DMAMUX1_BASE)

class DMA_t {
private:
    uint32_t ChnlN, ReqID;
    DMA_Channel_TypeDef *PChnl;
public:
    DMA_t(uint32_t AChnl, uint32_t AReqID,
            ftVoidPVoidW32 PIrqFunc = nullptr, void *PIrqParam = nullptr,
            uint32_t AIrqPrio = IRQ_PRIO_MEDIUM);
    void Init() const;
    void Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const;
    void SetPeriphAddr(volatile void* Addr) const { PChnl->CPAR  = (uint32_t)Addr; }
    void SetMemoryAddr(void* Addr) const { PChnl->CMAR  = (uint32_t)Addr; }
    void* GetMemoryAddr()                   const { return (void*)PChnl->CMAR; }
    void SetMode(uint32_t AMode)            const { PChnl->CCR = AMode; }
    void SetTransferDataCnt(uint16_t Cnt)   const { PChnl->CNDTR = Cnt; }
    uint16_t GetTransferDataCnt()           const { return PChnl->CNDTR; }

    void Enable()                           const { PChnl->CCR |= DMA_CCR_EN; }
    void Disable()                          const { PChnl->CCR &= ~DMA_CCR_EN; }
    void ClearIrq() const { DMA1->IFCR = 0x0EUL << (ChnlN-1); }
    void DisableAndClearIRQ() const {
        PChnl->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE | DMA_CCR_EN);
        ClearIrq();
    }
};
#endif

#if 1 // ======================== Flash and Option bytes =======================
namespace Flash {

void UnlockFlash();
void LockFlash();

uint8_t ErasePage(uint32_t PageAddress);
uint8_t ProgramDWord(uint32_t Address, uint64_t Data);

bool FirmwareIsLocked();
void LockFirmware();
void UnlockFirmware();

bool IwdgIsFrozenInStandby();
void IwdgFrozeInStandby();

}; // Namespace
#endif

#if 1 // ============================== IWDG ===================================
namespace Iwdg {

// Up to 32000 ms
void InitAndStart(uint32_t ms);

static inline void Reload() { IWDG->KR = 0xAAAA; }

static inline bool ResetOccured() {
    if(RCC->CSR & RCC_CSR_IWDGRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;   // Clear flags
        return true;
    }
    else return false;
}

void DisableInDebug();

};
#endif

#endif // KLLIBG070_H__
