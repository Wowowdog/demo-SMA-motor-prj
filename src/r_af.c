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
* File Name : r_af.c
* Version : 1.0
* Device(s) : RUMBA-S4H
* Description : This file implements control for AF(AFCNT) Module.
******************************************************************************/
/******************************************************************************
* History : DD.MM.YYYY Version Description
*           30.11.2015 1.00    First Release
******************************************************************************/

/******************************************************************************
Includes <System Includes> , "Project Includes"
******************************************************************************/
#include "r_macrodriver.h"
#include "r_ois.h"
#include "r_afcnt.h"
#include "r_af.h"

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Exported global variables (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
uint8_t  AfCtrlState;                                                   /* Cntrol State */
uint16_t AfTargetPos;                                                   /* Target Position */

/******************************************************************************
* Function Name: af_main
* Description : This function is the AF(AFCNT) main processing.
* Arguments : none
* Return Value : none
******************************************************************************/
void R_AF_main(void)
{
    if(CMD_REG->AFTARGETPOS != AfTargetPos)
    {
        AfTargetPos = CMD_REG->AFTARGETPOS;

        R_AFCNT_Set_Current(AfTargetPos);                               /* Set the Current */
    }

    switch(AfCtrlState)
    {
        case AF_DISABLE:
            if(CMD_REG->AFCTRL == AF_ENABLE)
            {
                R_AFCNT_Start();                                        /* Start the Drive */

                AfCtrlState = AF_ENABLE;
            }
            break;

        case AF_ENABLE:
            if(CMD_REG->AFCTRL == AF_DISABLE)
            {
                AfCtrlState = AF_DISABLE;

                R_AFCNT_Stop();                                         /* Stop the Drive */
            }
            break;

        default:
            break;
    }
}

/******************************************************************************
* Function Name: R_AF_Init
* Description : This function initializes the AF(AFCNT) module.
* Arguments : none
* Return Value : none
******************************************************************************/
void R_AF_Init(void)
{
    CMD_REG->AFCTRL      = AF_DISABLE;                                  /* Initialize of Command Register */
    CMD_REG->AFTARGETPOS = 0U;

    AfCtrlState          = CMD_REG->AFCTRL;                             /* Initialize of Global Variables */
    AfTargetPos          = CMD_REG->AFTARGETPOS;

    R_AFCNT_Set_Current(AfTargetPos);                                   /* Set the Current */
}
