/*
 * kl_libG070.cpp
 *
 *  Created on: 17 ���. 2020 �.
 *      Author: layst
 */

#include "kl_libG070.h"
#include "cmsis_gcc.h"

uint32_t AHBFreqHz, APBFreqHz;

/********************************************
arena;     total space allocated from system
ordblks;   number of non-inuse chunks
hblks;     number of mmapped regions
hblkhd;    total space in mmapped regions
uordblks;  total allocated space
fordblks;  total non-inuse space
keepcost;  top-most, releasable (via malloc_trim) space
**********************************************/
//void PrintMemoryInfo() {
//    struct mallinfo info = mallinfo();
//    Printf(
//            "total space allocated from system: %u\r"
//            "number of non-inuse chunks: %u\r"
//            "number of mmapped regions: %u\r"
//            "total space in mmapped regions: %u\r"
//            "total allocated space: %u\r"
//            "total non-inuse space: %u\r"
//            "top-most, releasable: %u\r",
//            info.arena, info.ordblks, info.hblks, info.hblkhd,
//            info.uordblks, info.fordblks, info.keepcost);
//}

#if 1 // ============================= Timer ===================================
void Timer_t::Init() const {
    if     (ITmr == TIM3)   { RCC->APBENR1 |= RCC_APBENR1_TIM3EN; }
    else if(ITmr == TIM6)   { RCC->APBENR1 |= RCC_APBENR1_TIM6EN; }
    else if(ITmr == TIM7)   { RCC->APBENR1 |= RCC_APBENR1_TIM7EN; }
    else if(ITmr == TIM14)  { RCC->APBENR2 |= RCC_APBENR2_TIM14EN; }
    else if(ITmr == TIM15)  { RCC->APBENR2 |= RCC_APBENR2_TIM15EN; }
    else if(ITmr == TIM16)  { RCC->APBENR2 |= RCC_APBENR2_TIM16EN; }
    else if(ITmr == TIM17)  { RCC->APBENR2 |= RCC_APBENR2_TIM17EN; }
}

//void Timer_t::Reset() const {
//    uint32_t cr1 = ITmr->CR1;
//    uint32_t cr2 = ITmr->CR2;
//    uint32_t smcr = ITmr->SMCR;
//    uint32_t smcr = ITmr->SMCR;
//    uint32_t dier = ITmr->DIER;
//    uint32_t ccmr1 = ITmr->CCMR1;
//    uint32_t ccmr2 = ITmr->CCMR2;
//    uint32_t ccer = ITmr->CCER;
//    uint32_t psc = ITmr->PSC;

//}

void Timer_t::SetupPrescaler(uint32_t PrescaledFreqHz) const {
    ITmr->PSC = (APBFreqHz / PrescaledFreqHz) - 1;
}

void Timer_t::SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const {
    // Figure out input timer freq
    uint32_t UpdFreqMax = APBFreqHz / (ITmr->ARR + 1);
    uint32_t div = UpdFreqMax / FreqHz;
    if(div != 0) div--;
//    Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
    ITmr->PSC = div;
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const {
    uint32_t UpdFreqMax = APBFreqHz / (ITmr->PSC + 1);
    uint32_t TopVal  = (UpdFreqMax / FreqHz);
    if(TopVal != 0) TopVal--;
    SetTopValue(TopVal);
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingBoth(uint32_t FreqHz) const {
    uint32_t Psc = (APBFreqHz / FreqHz) / 0x10000;
    ITmr->PSC = Psc;
    SetUpdateFrequencyChangingTopValue(FreqHz);
}
#endif

#if 1 // ============================== PWM ====================================
void PinOutputPWM_t::Init() const {
    Timer_t::Init();
    ITmr->BDTR = 0xC000;   // Main output Enable
    ITmr->ARR = ISetup.TopValue;
    // Setup Output
    uint16_t tmp = (ISetup.Inverted == invInverted)? 0b111 : 0b110; // PWM mode 1 or 2
    switch(ISetup.TimerChnl) {
        case 1:
            ITmr->CCMR1 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;
        case 2:
            ITmr->CCMR1 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;
        case 3:
            ITmr->CCMR2 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;
        case 4:
            ITmr->CCMR2 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;
        default: break;
    }
    Enable();

    // GPIO
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, ISetup.AF);
}
#endif

#if 1 // ================================= SPI =================================
void Spi_t::Setup(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA, int32_t Bitrate_Hz, BitNumber_t BitNumber) const {
    // Clocking
    if      (PSpi == SPI1) { RCC->APBENR2 |= RCC_APBENR2_SPI1EN; }
    else if (PSpi == SPI2) { RCC->APBENR1 |= RCC_APBENR1_SPI2EN; }
    // Bit number
    if(BitNumber == bitn16) PSpi->CR2 = 0b1111U << 8;  // 16 bit, RXNE generated when 16 bit is received
    else PSpi->CR2 = (0b0111U << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
    // Mode: Master, NSS software controlled and is 1, 8bit, NoCRC, FullDuplex
    PSpi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
    if(BitOrder == boLSB) PSpi->CR1 |= SPI_CR1_LSBFIRST;    // MSB/LSB
    if(CPOL == cpolIdleHigh) PSpi->CR1 |= SPI_CR1_CPOL;     // CPOL
    if(CPHA == cphaSecondEdge) PSpi->CR1 |= SPI_CR1_CPHA;   // CPHA
    // Baudrate
    int32_t div;
    div = APBFreqHz / Bitrate_Hz;
    SpiClkDivider_t ClkDiv = sclkDiv2;
    if     (div > 128) ClkDiv = sclkDiv256;
    else if(div > 64) ClkDiv = sclkDiv128;
    else if(div > 32) ClkDiv = sclkDiv64;
    else if(div > 16) ClkDiv = sclkDiv32;
    else if(div > 8)  ClkDiv = sclkDiv16;
    else if(div > 4)  ClkDiv = sclkDiv8;
    else if(div > 2)  ClkDiv = sclkDiv4;
    PSpi->CR1 |= ((uint16_t)ClkDiv) << 3;
}

void Spi_t::SetupSlave(BitOrder_t BitOrder, CPOL_t CPOL, CPHA_t CPHA, BitNumber_t BitNumber) const {
    // Clocking
    if      (PSpi == SPI1) { RCC->APBENR2 |= RCC_APBENR2_SPI1EN; }
    else if (PSpi == SPI2) { RCC->APBENR1 |= RCC_APBENR1_SPI2EN; }
    PSpi->CR1 = 0; // Mode: Slave, NSS hardware controlled, NoCRC, FullDuplex
    PSpi->CR2 = 0;
    // Bit number
    if(BitNumber == bitn16) PSpi->CR2 = 0b1111U << 8;  // 16 bit, RXNE generated when 16 bit is received
    else PSpi->CR2 = (0b0111U << 8) | SPI_CR2_FRXTH;   // 8 bit, RXNE generated when 8 bit is received
    // CPOL, CPHA, Bit Order
    if(CPOL == cpolIdleHigh)   PSpi->CR1 |= SPI_CR1_CPOL; // CPOL
    if(CPHA == cphaSecondEdge) PSpi->CR1 |= SPI_CR1_CPHA; // CPHA
    if(BitOrder == boLSB)      PSpi->CR1 |= SPI_CR1_LSBFIRST; // MSB/LSB
}

void Spi_t::Reset() const {
    uint32_t cr1 = PSpi->CR1;
    uint32_t cr2 = PSpi->CR2;
    if(PSpi == SPI1) {
        RCC->APBENR2 &= ~RCC_APBENR2_SPI1EN;
        RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
        RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;
        RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    }
    else if (PSpi == SPI2) {
        RCC->APBENR1 &= ~RCC_APBENR1_SPI2EN;
        RCC->APBRSTR1 |= RCC_APBRSTR1_SPI2RST;
        RCC->APBRSTR1 &= ~RCC_APBRSTR1_SPI2RST;
        RCC->APBENR1 |= RCC_APBENR1_SPI2EN;
    }
    PSpi->CR1 = cr1;
    PSpi->CR2 = cr2;
}

// ========================= I2S =========================
void I2S_t::SetupI2SMasterTx16Bit() const {
    // Clocking
    if      (PSpi == SPI1) { RCC->APBENR2 |= RCC_APBENR2_SPI1EN; }
    else if (PSpi == SPI2) { RCC->APBENR1 |= RCC_APBENR1_SPI2EN; }
    // I2S mode, master transmit, standard = i2s, clock idle low, datlen=16 bit, channel len=16 bit
    PSpi->I2SCFGR = SPI_I2SCFGR_I2SMOD | (0b10U << 8) | (0b00U << 4);
}

void I2S_t::Reset() const {
    uint32_t cr1 = PSpi->CR1;
    uint32_t cr2 = PSpi->CR2;
    if(PSpi == SPI1) {
        RCC->APBENR2 &= ~RCC_APBENR2_SPI1EN;
        RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
        RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;
        RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    }
    else if (PSpi == SPI2) {
        RCC->APBENR1 &= ~RCC_APBENR1_SPI2EN;
        RCC->APBRSTR1 |= RCC_APBRSTR1_SPI2RST;
        RCC->APBRSTR1 &= ~RCC_APBRSTR1_SPI2RST;
        RCC->APBENR1 |= RCC_APBENR1_SPI2EN;
    }
    PSpi->CR1 = cr1;
    PSpi->CR2 = cr2;
}

// IRQs
//static ftVoidVoid Spi1RxIrqHandler = nullptr;
//static ftVoidVoid Spi2RxIrqHandler = nullptr;

//void Spi_t::SetupRxIrqCallback(ftVoidVoid AIrqHandler) const {
//    if(PSpi == SPI1) Spi1RxIrqHandler = AIrqHandler;
//    else if(PSpi == SPI2) Spi2RxIrqHandler = AIrqHandler;
//}

extern "C" {
//void VectorCC() {   // SPI1
//    CH_IRQ_PROLOGUE();
//    chSysLockFromISR();
//    uint32_t SR = SPI1->SR;
//    if(SR & SPI_SR_RXNE and Spi1RxIrqHandler) Spi1RxIrqHandler();
//    chSysUnlockFromISR();
//    CH_IRQ_EPILOGUE();
//}
//
//void VectorD0() {   // SPI2
//    CH_IRQ_PROLOGUE();
//    chSysLockFromISR();
//    uint32_t SR = SPI2->SR;
//    if(SR & SPI_SR_RXNE and Spi2RxIrqHandler) Spi2RxIrqHandler();
//    chSysUnlockFromISR();
//    CH_IRQ_EPILOGUE();
//}

} // extern C

#endif

#if 1 // ============================== DMA ====================================
struct DmaIrqHandler_t {
    ftVoidPVoidW32 Handler = nullptr;
    void *Param = nullptr;
    uint32_t Prio = IRQ_PRIO_LOW;
};

static DmaIrqHandler_t DmaIrqHandler[DMA_CHNL_CNT];

DMA_t::DMA_t(uint32_t AChnl, uint32_t AReqID,
        ftVoidPVoidW32 PIrqFunc, void *PIrqParam, uint32_t AIrqPrio) :
    ChnlN(AChnl), ReqID(AReqID) {
    DmaIrqHandler[AChnl-1].Handler = PIrqFunc;
    DmaIrqHandler[AChnl-1].Param = PIrqParam;
    DmaIrqHandler[AChnl-1].Prio = AIrqPrio;
    if     (AChnl == 1) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel1_BASE;
    else if(AChnl == 2) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel2_BASE;
    else if(AChnl == 3) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel3_BASE;
    else if(AChnl == 4) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel4_BASE;
    else if(AChnl == 5) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel5_BASE;
    else if(AChnl == 6) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel6_BASE;
    else if(AChnl == 7) PChnl = (DMA_Channel_TypeDef *)DMA1_Channel7_BASE;
}


void DMA_t::Init() const {
    if(DmaIrqHandler[ChnlN-1].Handler != nullptr) {
        if(ChnlN == 1) {
            NVIC_SetPriority(DMA1_Channel1_IRQn, DmaIrqHandler[ChnlN-1].Prio);
            NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        }
        else if(ChnlN == 2 or ChnlN == 3) {
            NVIC_SetPriority(DMA1_Channel2_3_IRQn, DmaIrqHandler[ChnlN-1].Prio);
            NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
        }
        else {
            NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, DmaIrqHandler[ChnlN-1].Prio);
            NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
        }
    } // if irq
    PChnl->CCR = 0; // Reset value
    DMAMUX->CCR[ChnlN-1] &= ~0x3FUL;
    DMAMUX->CCR[ChnlN-1] |= ReqID;
}

void DMA_t::Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const {
    Init();
    SetPeriphAddr(PeriphAddr);
    SetMemoryAddr(MemAddr);
    SetMode(AMode);
    SetTransferDataCnt(Cnt);
}


// ==== IRQs ====
extern "C" {
void DMA1_Channel1_IRQHandler() {
    uint32_t flags = DMA1->ISR & 0xFUL; // Mask only needed bits
    DMA1->IFCR = DMA_IFCR_CGIF1;        // Clear all irq flags
    ftVoidPVoidW32 func = DmaIrqHandler[0].Handler;
    if(func) func(DmaIrqHandler[0].Param, flags >> 0);
}

void DMA1_Channel2_3_IRQHandler() {
    uint32_t flags = DMA1->ISR & 0xFF0UL;
    if(flags & DMA_ISR_GIF2) { // Global IRQ flag for second chnl
        DMA1->IFCR = DMA_IFCR_CGIF2;
        ftVoidPVoidW32 func = DmaIrqHandler[1].Handler;
        if(func) func(DmaIrqHandler[1].Param, (flags >> 4) & 0xFUL);
    }
    if(flags & DMA_ISR_GIF3) { // Global IRQ flag
        DMA1->IFCR = DMA_IFCR_CGIF3;
        ftVoidPVoidW32 func = DmaIrqHandler[2].Handler;
        if(func) func(DmaIrqHandler[2].Param, (flags >> 8) & 0xFUL);
    }
}

void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler() {
    uint32_t flags = DMA1->ISR & 0xFFFF000UL;
    if(flags & DMA_ISR_GIF4) { // Global IRQ flag
        DMA1->IFCR = DMA_IFCR_CGIF4;
        ftVoidPVoidW32 func = DmaIrqHandler[3].Handler;
        if(func) func(DmaIrqHandler[3].Param, (flags >> 12) & 0xFUL);
    }
    if(flags & DMA_ISR_GIF5) { // Global IRQ flag
        DMA1->IFCR = DMA_IFCR_CGIF5;
        ftVoidPVoidW32 func = DmaIrqHandler[4].Handler;
        if(func) func(DmaIrqHandler[4].Param, (flags >> 16) & 0xFUL);
    }
    if(flags & DMA_ISR_GIF6) { // Global IRQ flag
        DMA1->IFCR = DMA_IFCR_CGIF6;
        ftVoidPVoidW32 func = DmaIrqHandler[5].Handler;
        if(func) func(DmaIrqHandler[5].Param, (flags >> 20) & 0xFUL);
    }
    if(flags & DMA_ISR_GIF7) { // Global IRQ flag
        DMA1->IFCR = DMA_IFCR_CGIF7;
        ftVoidPVoidW32 func = DmaIrqHandler[6].Handler;
        if(func) func(DmaIrqHandler[6].Param, (flags >> 24) & 0xFUL);
    }
}

} // extern C
#endif

#if 1 // =========================== External IRQ ==============================
// IRQ handlers
extern "C" {
ftVoidVoid ExtiIrqHandler_0_1, ExtiIrqHandler_2_3, ExtiIrqHandler_4_15;

void EXTI0_1_IRQHandler() {
    EXTI->RPR1 = (1<<1) | (1<<0);
    EXTI->FPR1 = (1<<1) | (1<<0);
    if(ExtiIrqHandler_0_1 != nullptr) ExtiIrqHandler_0_1();
}

void EXTI2_3_IRQHandler() {
    EXTI->RPR1 = (1<<3) | (1<<2);
    EXTI->FPR1 = (1<<3) | (1<<2);
    if(ExtiIrqHandler_2_3 != nullptr) ExtiIrqHandler_2_3();
}

void EXTI4_15_IRQHandler() {
    EXTI->RPR1 = 0xFFF0;
    EXTI->FPR1 = 0xFFF0;
    if(ExtiIrqHandler_4_15 != nullptr) ExtiIrqHandler_4_15();
}

} // extern C
#endif

#if 1 // ======================== Flash and Option bytes =======================
namespace Flash {

// Wait for a Flash operation to complete or a TIMEOUT to occur
static uint8_t WaitForLastOperation(uint32_t Timeout) {
    while(FLASH->SR & FLASH_SR_BSY1) {
        if(!Timeout--) return retvTimeout;
    }
    return retvOk;
}

void UnlockFlash() {
    __disable_irq();
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    __enable_irq();
}
void LockFlash() {
    __disable_irq();
    if(WaitForLastOperation(63000000) == retvOk) FLASH->CR |= FLASH_CR_LOCK;
    __enable_irq();
}

void ClearErrFlags() {
    FLASH->SR |= FLASH_SR_OPTVERR | FLASH_SR_FASTERR |
            FLASH_SR_MISERR | FLASH_SR_PGSERR | FLASH_SR_SIZERR |
            FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR;
}

// Beware: use Page Address (0...63), not absolute address kind of 0x08003f00
uint8_t ErasePage(uint32_t PageAddress) {
    __disable_irq();
    uint8_t status = WaitForLastOperation(63000000);
    if(status == retvOk) {
        ClearErrFlags();    // Clear all error programming flags
        uint32_t Reg = FLASH->CR;
        Reg &= ~FLASH_CR_PNB;
        Reg |= (PageAddress << FLASH_CR_PNB_Pos) | FLASH_CR_PER;
        FLASH->CR = Reg;
        FLASH->CR |= FLASH_CR_STRT;
        status = WaitForLastOperation(63000000);
        FLASH->CR &= ~FLASH_CR_PER; // Disable the PageErase Bit
    }
    __enable_irq();
    return status;
}

uint8_t ProgramDWord(uint32_t Address, uint64_t Data) {
    __disable_irq();
    uint8_t status = WaitForLastOperation(63000000);
    if(status == retvOk) {
        ClearErrFlags();
        // Program Dword
        FLASH->CR |= FLASH_CR_PG;    // Enable flash writing
        *(volatile uint32_t*)Address = (uint32_t)Data;
        *(volatile uint32_t*)(Address + 4) = (uint32_t)(Data >> 32);
        status = WaitForLastOperation(63000000);
        if(FLASH->SR & FLASH_SR_EOP) FLASH->SR |= FLASH_SR_EOP; // Clear EOP if set
        FLASH->CR &= ~FLASH_CR_PG;          // Disable flash writing
    }
    __enable_irq();
    return status;
}

}; // Namespace
#endif

#if 1 // ========================= Time and Delay ==============================
void Time_t::Init() {
    Timer_t::Init();
    SetTopValue(0xFFFF);
    SetupPrescaler(1000); // 1000 Hz => 1ms tick
    Enable();
}

void Time_t::Wait(uint16_t Time_ms) {
    uint16_t Start = GetCounter();
    while(ElapsedSince(Start) < Time_ms);
}

uint32_t Time_t::ElapsedSince(uint32_t Start) {
    return ((uint16_t)GetCounter()) - (uint16_t)Start;
}
#endif

#if 1 // ============================== IWDG ===================================
namespace Iwdg {
enum Pre_t {
    iwdgPre4 = 0x00,
    iwdgPre8 = 0x01,
    iwdgPre16 = 0x02,
    iwdgPre32 = 0x03,
    iwdgPre64 = 0x04,
    iwdgPre128 = 0x05,
    iwdgPre256 = 0x06
};

void DisableInDebug() {
    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_IWDG_STOP;
}

static void Enable() { IWDG->KR = 0xCCCC; }
static void EnableAccess() { IWDG->KR = 0x5555; }

static void SetPrescaler(Pre_t Prescaler) { IWDG->PR = (uint32_t)Prescaler; }
static void SetReload(uint16_t Reload) { IWDG->RLR = Reload; }

void SetTimeout(uint32_t ms) {
    EnableAccess();
    SetPrescaler(iwdgPre256);
    uint32_t Count = (ms * (LSI_FREQ_HZ/1000UL)) / 256UL;
    if(Count > 0xFFF) Count = 0xFFF;
    SetReload(Count);
    Reload();   // Reload and lock access
}

void InitAndStart(uint32_t ms) {
    EnableLSI();    // Start LSI
    SetTimeout(ms); // Start IWDG
    Enable();
}

}; // Namespace
#endif

