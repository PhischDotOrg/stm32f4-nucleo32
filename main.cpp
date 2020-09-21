/*-
 * $Copyright$
-*/
#include <common/Infrastructure.hpp>
#include <phisch/log.h>

/* for vTaskStartScheduler */
#include <FreeRTOS.h> 
#include <FreeRTOS/include/task.h>

#include <stm32/Cpu.hpp>

#include <stm32/Pll.hpp>
#include <stm32/Pwr.hpp>
#include <stm32/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Nvic.hpp>

#include <gpio/GpioAccess.hpp>
#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32/Uart.hpp>
#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

#include <tasks/Heartbeat.hpp>

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static const constexpr stm32::PllCfg pllCfg = {
    .m_pllSource        = stm32::PllCfg::PllSource_t::e_PllSourceMSI,
    .m_msiRange         = stm32::PllCfg::MSIRange_t::e_MSIRange_4MHz,
    .m_hseSpeedInHz     = 0,

    /* Main PLL */
    .m_pllM             = 1,
    .m_pllN             = 40,
    .m_pllP             = stm32::PllCfg::PllP_t::e_PllP_Disabled,
    .m_pllQ             = stm32::PllCfg::PllQ_t::e_PllQ_Disabled,
    .m_pllR             = stm32::PllCfg::PllR_t::e_PllR_Div2,

    /* Serial Audio Interface (SAI) PLL */
    .m_pllSaiN          = 24,
    .m_pllSaiP          = stm32::PllCfg::PllP_t::e_PllP_Disabled,
    .m_pllSaiQ          = stm32::PllCfg::PllQ_t::e_PllQ_Disabled,
    .m_pllSaiR          = stm32::PllCfg::PllR_t::e_PllR_Div2,

    /* System and AHP / APB Bus Clocks */
    .m_sysclkSource     = stm32::PllCfg::SysclkSource_t::e_SysclkPLL,
    .m_ahbPrescaler     = stm32::PllCfg::AHBPrescaler_t::e_AHBPrescaler_None,
    .m_apb1Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None,
    .m_apb2Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None,
};

static stm32::Scb                       scb(SCB);
static stm32::Nvic                      nvic(NVIC, scb);

static stm32::Pwr                       pwr(PWR);
static stm32::Flash                     flash(FLASH);
static stm32::Rcc                       rcc(RCC, pllCfg, flash, pwr);

/*******************************************************************************
 * GPIO Engine Handlers 
 ******************************************************************************/
static stm32::Gpio::A                   gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static stm32::Gpio::B                   gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::AlternateFnPin             g_mco1(gpio_engine_A, 8);
static gpio::DigitalOutPin              g_led_green(gpio_engine_B, 3);

/*******************************************************************************
 * UART
 ******************************************************************************/
static gpio::AlternateFnPin             uart_tx(gpio_engine_A, 2);
static gpio::AlternateFnPin             uart_rx(gpio_engine_A, 15);
static stm32::Uart::Usart2<gpio::AlternateFnPin>    uart_access(rcc, uart_rx, uart_tx);
uart::UartDevice                        g_uart(&uart_access);

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::HeartbeatT<decltype(g_led_green)> heartbeat_gn("hrtbt_g", g_led_green, 3, 500);

/*******************************************************************************
 *
 ******************************************************************************/
const uint32_t SystemCoreClock = pllCfg.getSysclkSpeedInHz();

static_assert(pllCfg.isValid() == true,                            "PLL Configuration is not valid!");
static_assert(SystemCoreClock               == 80 * 1000 * 1000,   "Expected System Clock to be at 80 MHz!");
static_assert(pllCfg.getAhbSpeedInHz()      == 80 * 1000 * 1000,   "Expected AHB to be running at 80 MHz!");
static_assert(pllCfg.getApb1SpeedInHz()     == 80 * 1000 * 1000,   "Expected APB1 to be running at 80 MHz!");
static_assert(pllCfg.getApb2SpeedInHz()     == 80 * 1000 * 1000,   "Expected APB2 to be running at 80 MHz!");

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

int
main(void) {
    rcc.setMCO(g_mco1, decltype(rcc)::MCO1Output_e::e_PLL, decltype(rcc)::MCOPrescaler_t::e_MCOPre_5);

    uart_access.setBaudRate(decltype(uart_access)::BaudRate_e::e_115200);

    const unsigned sysclk = pllCfg.getSysclkSpeedInHz() / 1000;
    const unsigned ahb    = pllCfg.getAhbSpeedInHz() / 1000;
    const unsigned apb1   = pllCfg.getApb1SpeedInHz() / 1000;
    const unsigned apb2   = pllCfg.getApb2SpeedInHz() / 1000;

    PrintStartupMessage(sysclk, ahb, apb1, apb2);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        PHISCH_LOG("FATAL: Capture Error!\r\n");
        goto bad;
    }

    PHISCH_LOG("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

bad:
    PHISCH_LOG("FATAL ERROR!\r\n");
    while (1) ;

    return (0);
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
