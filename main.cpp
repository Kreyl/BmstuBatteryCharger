#include "board.h"
#include <inttypes.h>
#include "kl_libG070.h"
#include "uartG070.h"
#include "shell.h"
#include "AdcG070.h"

#if 1 // ======================== Variables & prototypes =======================
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void ClockInit();
void ITask();

Time_t Time{TIME_TIMER};
#endif

int main(void) {
    __disable_irq();
    ClockInit();
    __enable_irq();

    // ==== Init hardware ====
    PinSetupOut(DBG_PIN, omPushPull);
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
//    Time.Init();

    // Setup TIM1
    PinSetupAlterFunc(GPIOA, 7, omPushPull, pudNone, AF2, psHigh); // CH1N
    PinSetupAlterFunc(GPIOA, 8, omPushPull, pudNone, AF2, psHigh); // CH1
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
    TIM1->CR1 |=  TIM_CR1_ARPE;
    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE;

    // PWM mode 2
    TIM1->CCMR1 = 0b110UL << 4;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P;

    TIM1->ARR = 6400-1;
    TIM1->CCR1 = 999;

    TIM1->CR1 |=  TIM_CR1_CEN; // Enable

    // Main cycle
    ITask();
}

void OnUartCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    if(PCmd->NameIs("Ping")) PShell->Ok();

    else if(PCmd->NameIs("CCR")) {
        uint32_t v;
        if(PCmd->GetNext(&v) == retvOk) {
            TIM1->CCR1 = v;
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("DT")) {
        uint32_t v;
        if(PCmd->GetNext(&v) == retvOk) {
            TIM1->BDTR = (TIM1->BDTR & 0xFFFFFF00) | (v & 0xFFUL);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("ARR")) {
        uint32_t v;
        if(PCmd->GetNext(&v) == retvOk) {
            TIM1->ARR = v;
            PShell->Ok();
        }
        else PShell->BadParam();
    }
}

__attribute__((__noreturn__))
void ITask() {
    while(true) {
        if(Uart.GetRcvdCmd() == retvOk) OnUartCmd(&Uart);
    } // while true
}
void ClockInit() {
    uint32_t tmpreg;
    // Reset everything
    RCC->AHBRSTR = 0xFFFFFFFF;
    RCC->AHBRSTR = 0;
    (void)RCC->AHBRSTR;
    RCC->APBRSTR1 = 0xFFFFFFFF;
    RCC->APBRSTR1 = 0;
    (void)RCC->APBRSTR1;
    RCC->APBRSTR2 = 0xFFFFFFFF;
    RCC->APBRSTR2 = 0;
    (void)RCC->APBRSTR2;

    // Enable syscfg and pwr
    EnableAPB2Clk(RCC_APBENR2_SYSCFGEN);
    EnableAPB1Clk(RCC_APBENR1_PWREN);
    // Set Flash latency
    tmpreg = FLASH->ACR;
    tmpreg &= ~FLASH_ACR_LATENCY;
    tmpreg |= 0b010UL; // Flash latency: two wait states
    tmpreg |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN; // Enable instruction cache and prefetch
    FLASH->ACR = tmpreg;

    // === PLL === PLLSRC = HSI = 16MHz
//    uint32_t M=1, N=12, R=3, P=2; // 64 MHz SysClk, 24MHz I2S
    uint32_t M=1, N=20, R=5, P=3; // 64 MHz SysClk, 17MHz I2S
//    uint32_t M=2, N=30, R=4, P=2; // 60 MHz SysClk, 120 PLLP, 20MHz I2S
    RCC->PLLCFGR = ((R - 1UL) << 29) | ((P - 1UL) << 17) | (N << 8) | ((M-1UL) << 4) | (0b10UL << 0);
    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable R output

    // Wait until PLL started
    while(!(RCC->CR & RCC_CR_PLLRDY));
    // Switch to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | 0b010UL; // PLLR is sys clk
    // Wait until done
    while((RCC->CFGR & RCC_CFGR_SWS) != (0b010UL << 3));
    // Enable DMA clock
    EnableAHBClk(RCC_AHBENR_DMA1EN);
    // ADC clock is SYSCLK: ADCSEL = 0
    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL;
    AHBFreqHz = 64000000;
    APBFreqHz = 64000000;
}
