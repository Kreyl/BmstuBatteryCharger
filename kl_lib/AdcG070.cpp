/*
 * AdcG070.cpp
 *
 *  Created on: 17 мая 2023 г.
 *      Author: layst
 */

#include "AdcG070.h"

namespace Adc {

#if ADC_DMA_ENABLE
static DMA_t DmaAdc{ADC_DMA_CHNL, ADC_DMA_REQID, AdcDmaIrqHandler, nullptr, IRQ_PRIO_MEDIUM};

void SetupAndEnableDMA(int32_t *PBuf, uint32_t ACnt) {
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
    // Setup DMA
    DmaAdc.Init(&ADC1->DR, PBuf,
            (DMA_PRIORITY_HIGH | DMA_MSIZE_32_BIT | DMA_PSIZE_16_BIT | DMA_MEM_INC | DMA_DIR_PER2MEM | DMA_CIRC | DMA_TCIE),
            ADC_CHNL_CNT);
    DmaAdc.Enable();
}
#endif

void Init() {
    EnableAPB2Clk(RCC_APBENR2_ADCEN);
//    ADC->CCR = 0b0010UL << 18;      // Pescaler=4 (64/4 = 16)
//    ADC1->CFGR2 = 0b10UL << 30;     // Clock: PCLK/4
    ADC->CCR = 0b0001UL << 18;      // Pescaler=2 (64/2 = 32)
    ADC1->CFGR2 = 0b01UL << 30;     // Clock: PCLK/2
//    ADC1->SMPR = 0b111;             // Sampling time: 160 ADC cycles
//    ADC1->SMPR = 0b110;             // Sampling time: 79.5 ADC cycles
//    ADC1->SMPR = 0b101;             // Sampling time: 39.5 ADC cycles
//    ADC1->SMPR = 0b100;             // Sampling time: 19.5 ADC cycles
//    ADC1->SMPR = 0b011;             // Sampling time: 12.5 ADC cycles
    ADC1->SMPR = 0b000;             // Sampling time: 1.5 ADC cycles
    ADC1->CR = ADC_CR_ADVREGEN;     // Enable voltage reg
    DelayLoop(36000);               // Let it stabilize
    ADC1->CR |= ADC_CR_ADCAL;       // Calibrate
    while(ADC1->CR & ADC_CR_ADCAL); // Wait until ADCAL become 0
    ADC1->CFGR1 &= ~ADC_CFGR1_CHSELRMOD; // 0: Each bit of the ADC_CHSELR register enables an input
    ADC1->ISR |= ADC_ISR_ADRDY;     // Clear ADRDY flag
    ADC1->CR |= ADC_CR_ADEN;        // Enable ADC
    while((ADC1->ISR | ADC_ISR_ADRDY) == 0); // Wait until ADRDY become 1
}

void SetupOversampling(OversampRatio_t Ratio, uint32_t Shift) {
    uint32_t tmp = ADC1->CFGR2 & (~(ADC_CFGR2_OVSR | ADC_CFGR2_OVSS));
    tmp |= ((uint32_t)Ratio << 2) | (Shift << 5) | ADC_CFGR2_OVSE; // Enable oversampler
    ADC1->CFGR2 = tmp;
}

void EnableChnl(uint32_t ChnlN) {
    ADC1->ISR |= ADC_ISR_CCRDY; // Clear Channel Configuration Ready flag
    ADC1->CHSELR |= 1 << ChnlN; // Set bit which enables channel
    if     (ChnlN == 12) ADC->CCR |= ADC_CCR_TSEN;
    else if(ChnlN == 13) ADC->CCR |= ADC_CCR_VREFEN;
    else if(ChnlN == 14) ADC->CCR |= ADC_CCR_VBATEN;
    while((ADC1->ISR & ADC_ISR_CCRDY) == 0); // wait until applied
}

void EnableConvDoneIRQ(uint32_t IrqPrio) {
    NVIC_SetPriority(ADC1_IRQn, IrqPrio);
    NVIC_EnableIRQ(ADC1_IRQn);
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;  // clear flags
    ADC1->IER = ADC_IER_EOCIE;
}

void EnableSequenceDoneIRQ(uint32_t IrqPrio) {
    NVIC_SetPriority(ADC1_IRQn, IrqPrio);
    NVIC_EnableIRQ(ADC1_IRQn);
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;  // clear flags
    ADC1->IER = ADC_IER_EOSEQIE;
}

void StartContinuous() {
    ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD; // en continuous, overwrite old result
    ADC1->CR |= ADC_CR_ADSTART;
}

#if defined ADC_TIMER
Timer_t ISamplingTmr{ADC_TIMER};

void SetupConvOnTimer(uint32_t FreqHz) {
    ISamplingTmr.Init();
    ISamplingTmr.SetUpdateFrequencyChangingBoth(FreqHz);
    ISamplingTmr.SelectMasterMode(mmUpdate);
    ISamplingTmr.Enable();

    Printf("Psc=%u; arr=%u\r", TIM6->PSC, TIM6->ARR);

    uint32_t tmp = ADC1->CFGR1 & ~(ADC_CFGR1_EXTEN | ADC_CFGR1_EXTSEL);
    if(ADC_TIMER == TIM6) tmp |= (0b01UL << 10) | (0b101UL << 6); // Ext trg on rising, TIM6_TRGO
    ADC1->CFGR1 = tmp;
}
#endif

void Start() {
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;  // clear flags
    ADC1->CR |= ADC_CR_ADSTART;
}

uint32_t GetVref_mv(int32_t aVref_adc) {
    return ((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / aVref_adc;
}

int32_t GetTemperature(int32_t at_adc, int32_t aVref_adc) {
    return ((((int32_t)((at_adc * GetVref_mv(aVref_adc)) / TEMPSENSOR_CAL_VREFANALOG) - (int32_t) *TEMPSENSOR_CAL1_ADDR) \
            * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)) / ((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR)) \
                    + TEMPSENSOR_CAL1_TEMP;
}


// Weak Irq handler
__attribute__((weak))
void IrqEocHanderI(uint32_t flags) { }
__attribute__((weak))
void IrqEosHanderI(uint32_t flags) { }

} // namespace

extern "C"
void ADC1_IRQHandler() {
//    Printf("Irq \r");
    uint32_t flags = ADC1->ISR;
    // End of conversion
    if(flags & ADC_ISR_EOC) Adc::IrqEocHanderI(flags);
    // End of sequence: reset pointer
//    if(flags & ADC_ISR_EOS) {
//        ADC1->ISR = ADC_ISR_EOS; // Clear flag
//        Adc::IrqEosHanderI(flags);
//    }
}
