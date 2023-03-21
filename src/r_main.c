/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* 2015 Renesas Electronics Corporation All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name : r_main.c
* Version : 1.0
* Device(s) : RUMBA-S4H
* Description : OIS main function
******************************************************************************/
/******************************************************************************
* History : DD.MM.YYYY Version Description
*           30.11.2015 1.00    First Release
******************************************************************************/

/******************************************************************************
Includes <System Includes> , "Project Includes"
******************************************************************************/
#include "RUMBA_sfr.h"
#include "r_macrodriver.h"
#include "r_ois.h"
#include "r_serial.h"
#include "r_clkc.h"
#include "r_systeminit.h"
#include "r_data_flash.h"
#include "r_af.h"
#include "sma_r.h"

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
extern unsigned int Image$$PROCESS_STACK$$ZI$$Limit;

/******************************************************************************
Exported global variables (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/

/******************************************************************************
* Function Name: main
* Description : This function implements main function.
* Arguments : none
* Return Value : none
******************************************************************************/

__declspec(noreturn) void main(void)
{
    register unsigned int SP_PROCESS __asm("psp");
    register unsigned int CONTROL __asm("control");

    /* Initialize Process Stack Pointer using linker-generated symbol from scatter-file */
    SP_PROCESS = (unsigned int) &Image$$PROCESS_STACK$$ZI$$Limit;

    /* Change Thread mode to use the Process Stack */
    CONTROL = CONTROL | 2;

    /* Flush and refill pipeline before proceeding */
    __isb(0xf);

    FLC->FLCUSROPT2.LWORD |= LVDSETTING_DISABLE;

    /* Select External clock */
    R_CLKC_SetExt();

    /* Data Flash write use DSP DATA Memory */
    R_CLKC_SupplyDSP();

    /* initialize of data flash */
    R_DATAFLASH_Init();

    /* initialize of command register */
    R_OIS_Init_CommandReg();

    R_Systeminit();
    R_OIS_Init();
    R_IICA0_Enable();
    R_AF_Init();
    A_SMA_Init_Variable();

    for( ; ; )
    {

        R_IICA0_UpdateCommandReg();
        /* This function is the OIS main processing. */
        R_OIS_main();
        /* This function is the AF(AFCNT) main processing. */
        R_AF_main();
        R_OIS_Get_FWInfo();
    }
}

