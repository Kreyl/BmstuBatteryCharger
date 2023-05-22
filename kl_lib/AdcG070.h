/*
 * AdcG070.h
 *
 *  Created on: 17 мая 2023 г.
 *      Author: layst
 */

#ifndef KL_LIB_ADCG070_H_
#define KL_LIB_ADCG070_H_

#include <inttypes.h>
#include "board.h"
#include "kl_libG070.h"

/*
 * t sens requires 5 us sampling time. Given ADC clk = 16MHz, sampling time must be 80 cycles
 */
#define ADC_CHNL_TSNS   12
#define ADC_CHNL_VREF   13
/* Internal voltage reference VrefInt */
#define VREFINT_CAL_ADDR                   ((uint16_t*) (0x1FFF75AAUL)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VREF                   ( 3000UL)                    /* Analog voltage reference (Vref+) voltage with which VrefInt has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
/* Temperature sensor */
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FFF75A8UL)) /* Internal temperature sensor, address of parameter TS_CAL1: On STM32G0, temperature sensor ADC raw data acquired at temperature  30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FFF75CAUL)) /* Internal temperature sensor, address of parameter TS_CAL2: On STM32G0, temperature sensor ADC raw data acquired at temperature 130 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL1_TEMP               (( int32_t)   30)            /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL2_TEMP               (( int32_t)  130)            /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL_VREFANALOG          ( 3000UL)                    /* Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */

namespace Adc {

enum OversampRatio_t {ratio2x=0, ratio4x=1, ratio8x=2, ratio16x=3, ratio32x=4, ratio64x=5, ratio128x=6, ratio256x=7};

#if defined ADC_TIMER
void SetupConvOnTimer(uint32_t FreqHz);
#endif

void Init();
void SetupOversampling(OversampRatio_t Ratio, uint32_t Shift);
void EnableChnl(uint32_t ChnlN);

void EnableConvDoneIRQ(uint32_t IrqPrio = IRQ_PRIO_LOW);
void EnableSequenceDoneIRQ(uint32_t IrqPrio = IRQ_PRIO_LOW);

#if ADC_DMA_ENABLE
void SetupAndEnableDMA(int32_t *PBuf, uint32_t ACnt);
void AdcDmaIrqHandler(void* p, uint32_t flags);
#endif

void StartContinuous();
void Start();
uint32_t GetVref_mv(int32_t aVref_adc);
int32_t GetTemperature(int32_t at_adc, int32_t aVref_adc);
int8_t GetTemperature();

// Irq handler
void IrqEocHanderI(uint32_t flags);
void IrqEosHanderI(uint32_t flags);
} // namespace

#endif /* KL_LIB_ADCG070_H_ */
