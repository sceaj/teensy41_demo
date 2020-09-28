/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    board.c
 * @brief   Board initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include <stdint.h>
#include "board.h"

#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "usb.h"
#include "usb_phy.h"

#define BOARD_DEBUG_USBCDC_BAUDRATE  115200
#define BOARD_DEBUG_USBCDC_INSTANCE  kUSB_ControllerEhci0
#define BOARD_DEBUG_USBCDC_PORT_TYPE kSerialPort_UsbCdc
#define BOARD_DEBUG_USBCDC_CLK_FREQ  BOARD_BOOTCLOCKRUN_USBPHY1_CLK


#define BOARD_DEBUG_UART_BAUDRATE    115200
#define BOARD_DEBUG_UART_INSTANCE    1
#define BOARD_DEBUG_UART_PORT_TYPE   kSerialPort_Uart
#define BOARD_DEBUG_UART_CLK_FREQ    300000000UL

#if (defined(SERIAL_PORT_TYPE_UART) && (SERIAL_PORT_TYPE_UART > 0U))
#define BOARD_DEBUG_BAUDRATE         BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_INSTANCE         BOARD_DEBUG_UART_INSTANCE
#define BOARD_DEBUG_PORT_TYPE        BOARD_DEBUG_UART_PORT_TYPE
#define BOARD_DEBUG_PORT_CLK_FREQ    BOARD_DEBUG_UART_CLK_FREQ
#else
#define BOARD_DEBUG_BAUDRATE         BOARD_DEBUG_USBCDC_BAUDRATE
#define BOARD_DEBUG_INSTANCE         BOARD_DEBUG_USBCDC_INSTANCE
#define BOARD_DEBUG_PORT_TYPE        BOARD_DEBUG_USBCDC_PORT_TYPE
#define BOARD_DEBUG_PORT_CLK_FREQ    BOARD_DEBUG_USBCDC_CLK_FREQ
#endif

void USB_DeviceClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    if (BOARD_DEBUG_INSTANCE == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(BOARD_DEBUG_INSTANCE, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void BOARD_InitDebugConsole(void) {
	// The following function was needed to make USB work
	// I think it may be because something is not correctly initialized in the BOARD_InitBootClocks()
	// i.e. I missed something in ConfigTools/Clocks
	USB_DeviceClockInit();
    DbgConsole_Init(BOARD_DEBUG_INSTANCE, BOARD_DEBUG_BAUDRATE, BOARD_DEBUG_PORT_TYPE, BOARD_DEBUG_PORT_CLK_FREQ);
}
