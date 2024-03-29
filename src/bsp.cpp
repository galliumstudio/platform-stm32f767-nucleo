/*******************************************************************************
 * Copyright (C) 2018 Gallium Studio LLC (Lawrence Lo). All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Alternatively, this program may be distributed and modified under the
 * terms of Gallium Studio LLC commercial licenses, which expressly supersede
 * the GNU General Public License and are specifically designed for licensees
 * interested in retaining the proprietary status of their code.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Contact information:
 * Website - https://www.galliumstudio.com
 * Source repository - https://github.com/galliumstudio
 * Email - admin@galliumstudio.com
 ******************************************************************************/

#include <string.h>
#include "qpcpp.h"
#include "bsp.h"

Q_DEFINE_THIS_FILE

// Define this to enable debug print before UartAct objects are initialized.
// Debug messages are printed using HAL directly without DMA. This slows down the
// boot up process but allows all debug message to be seen since boot up.
#define ENABLE_BSP_PRINT

static volatile uint32_t idleCnt = 0;

static UART_HandleTypeDef usart;

/* top of stack (highest address) defined in the linker script -------------*/
extern int _estack;

static void InitUart() {
    // USART3 (TX=PD8, RX=PD9) is used as the virtual COM port in ST-Link.
    __HAL_RCC_USART3_CLK_ENABLE();          // Customize.
    __GPIOD_CLK_ENABLE();                   // Customize.
    GPIO_InitTypeDef  gpioInit;
    gpioInit.Pin       = GPIO_PIN_8;        // Customize.
    gpioInit.Mode      = GPIO_MODE_AF_PP;   // Customize.
    gpioInit.Alternate = GPIO_AF7_USART3;   // Customize.
    gpioInit.Pull      = GPIO_PULLUP;
    gpioInit.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &gpioInit);
    usart.Instance = USART3;                // Customize.
    usart.Init.BaudRate = 115200;
    usart.Init.WordLength = UART_WORDLENGTH_8B;
    usart.Init.StopBits = UART_STOPBITS_1;
    usart.Init.Parity = UART_PARITY_NONE;
    usart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    usart.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&usart);
}

static void WriteUart(char const *buf, uint32_t len) {
    HAL_UART_Transmit(&usart, (uint8_t *)buf, len, 0xFFFF);
}

void BspInit() {
    // STM32F7xx HAL library initialization
    HAL_Init();

#ifdef ENABLE_BSP_PRINT
    InitUart();
#endif // ENABLE_BSP_PRINT

    char *testStr = "BspInit success\n\r";
    BspWrite(testStr, strlen(testStr));
}

void BspWrite(char const *buf, uint32_t len) {
#ifdef ENABLE_BSP_PRINT
     WriteUart(buf, len);
#endif
}

// Trace functions used by exception_handlers.c
// BspInitTrace() must be called before BspTrace() is called.
// Must be declared as C functions.
extern "C" void BspInitTrace() {
    InitUart();
}
extern "C" void BspTrace(char const *buf, uint32_t len) {
    WriteUart(buf, len);
    WriteUart("\r", 1);
}

uint32_t GetSystemMs() {
    return HAL_GetTick() * BSP_MSEC_PER_TICK;
}

uint32_t GetIdleCnt() {
    uint32_t cnt = idleCnt;
    idleCnt = 0;
    return cnt;
}

// Override the one defined in stm32f7xx_hal.c.
// Callback by HAL_Init() called from BspInit().
// Initialize SysTick interrupt as required by some HAL functions.
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    (void)TickPriority;
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);
    return HAL_OK;
}

// namespace QP **************************************************************
namespace QP {

// QF callbacks ==============================================================
void QF::onStartup(void) {
    // assigning all priority bits for preemption-prio. and none to sub-prio.
    NVIC_SetPriorityGrouping(0U);

    // Note SysTick has been initialized in HAL_InitTick(), which is called back
    // by HAL_Init() called from BspInit().
    // set priorities of ALL ISRs used in the system, see NOTE00
    //
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    // DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    //NVIC_SetPriority(EXTI0_1_IRQn,   DPP::EXTI0_1_PRIO);
    // ...

    // enable IRQs...
    //NVIC_EnableIRQ(EXTI0_1_IRQn);
}
//............................................................................
void QF::onCleanup(void) {
}
//............................................................................
void QXK::onIdle(void) {
    // toggle the User LED on and then off (not enough LEDs, see NOTE01)
    QF_INT_DISABLE();
    //GPIOA->BSRR |= (LED_LD2);        // turn LED[n] on
    //GPIOA->BSRR |= (LED_LD2 << 16);  // turn LED[n] off
    idleCnt++;
    QF_INT_ENABLE();

#if defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M3 MCU.
    //
    // !!!CAUTION!!!
    // The WFI instruction stops the CPU clock, which unfortunately disables
    // the JTAG port, so the ST-Link debugger can no longer connect to the
    // board. For that reason, the call to __WFI() has to be used with CAUTION.
    //
    // NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    // reset the board, then connect with ST-Link Utilities and erase the part.
    // The trick with BOOT(0) is it gets the part to run the System Loader
    // instead of your broken code. When done disconnect BOOT0, and start over.
    //
    //__WFI();   Wait-For-Interrupt
#endif
}

//............................................................................
extern "C" void Q_onAssert(char const * const module, int loc) {
    //
    // NOTE: add here your application-specific error handling
    //
    // Gallium
    InitUart();
    char buf[100];
    snprintf(buf, sizeof(buf), "ASSERT FAILED in %s at line %d\n\r", module, loc);
    WriteUart(buf, strlen(buf));
    QF_INT_DISABLE();
    for (;;) {
    }
    //NVIC_SystemReset();
}


/*****************************************************************************
* The function assert_failed defines the error/assertion handling policy
* for the application. After making sure that the stack is OK, this function
* calls Q_onAssert, which should NOT return (typically reset the CPU).
*
* NOTE: the function Q_onAssert should NOT return.
*****************************************************************************/
extern "C" __attribute__ ((naked))
void assert_failed(char const *module, int loc) {
    /* re-set the SP in case of stack overflow */
    __asm volatile (
        "  MOV sp,%0\n\t"
        : : "r" (&_estack));
    Q_onAssert(module, loc); /* call the application-specific QP handler */

    for (;;) { /* should not be reached, but just in case loop forever... */
    }
}


} // namespace QP

