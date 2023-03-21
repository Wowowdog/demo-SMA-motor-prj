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
* File Name : r_ois.c
* Version : 1.0
* Device(s) : RUMBA-S4H
* Description : This file implements control for OIS firmware.
******************************************************************************/
/******************************************************************************
* History : DD.MM.YYYY Version Description
*           30.11.2015 1.00    First Release
******************************************************************************/

/******************************************************************************
Includes <System Includes> , "Project Includes"
******************************************************************************/
#include "r_macrodriver.h"
#include "r_config.h"
#include "RUMBA_sfr.h"
#include "r_userdefine.h"
#include "r_ois.h"
#include "r_adon.h"
#include "r_tau.h"
#include "r_oiscnt.h"
#include "r_green_dsp.h"
#include "r_gyro.h"
#include "r_data_flash.h"
#include "r_anacnt.h"
#if(FEEDBACK_SELECTION == HALL)
	#include "r_hall.h"
#endif
#include "r_serial.h"
#include "version.h"
#include "r_gyrocom.h"
#include "sma_r.h"

/******************************************************************************
Macro definitions
******************************************************************************/
#define _80_TESTMON_REQ         (0x80U)
#define _80_TESTMON_REQ_MASK    (0x80U)
#define INIT_OISMODE            (OIS_MODE_0)
#define ANACNT_TEST_MON         (*(volatile uint16_t*)(0x50000376U))  /* AnalogControler MonitorSetting Reg */

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
extern uint16_t xcenter;
extern uint16_t ycenter;

extern int16_t xgyroraw;
extern int16_t ygyroraw;
extern uint16_t xgyrocalc;
extern uint16_t ygyrocalc;
extern uint8_t adcRequestFlag;
extern const stOISParam initOisParam;

/******************************************************************************
Exported global variables (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
static void R_OIS_DrvModeSet(void);
static ois_state_e R_OIS_ChangeState(void);
static void R_OIS_SetTestMonRegister(void);
static void R_OIS_Set_ComRegParamToFWData(void);
static void R_OIS_ParamInit(void);

static uint8_t CurrOisDrvMode;
static ois_switch_e pwmDutyFixedState;

uint32_t FWUPdateArg;
BootRomEntry pROMEntryTable;
uint32_t*  pAddress;
uint32_t temp=0;

volatile command_reg_t commandReg;

/******************************************************************************
* Function Name: R_OIS_DrvModeSet
* Description : Change OIS Drive Mode
* Arguments : none
* Return Value : none
******************************************************************************/
static void R_OIS_DrvModeSet( void )
{
    if( (uint8_t)CMD_REG->OISDRIVEMODE == DMODE_LINEAR )  // PWM->Linear
    {
        CurrOisDrvMode = DMODE_LINEAR;
    }
    else if( (uint8_t)CMD_REG->OISDRIVEMODE == DMODE_PWM) // Linear -> PWM
    {
        CurrOisDrvMode = DMODE_PWM;
    }
    else
    {
    	CurrOisDrvMode = DMODE_PIEZO;
    }
}

/******************************************************************************
* Function Name: R_OIS_DrvModeGet
* Description : Get OIS Drive Mode
* Arguments : none
* Return Value : Linear or PWM
******************************************************************************/
uint8_t R_OIS_GetDrvMode( void )
{
    return( CurrOisDrvMode );
}

/******************************************************************************
* Function Name: R_OIS_Init
* Description : OIS Initialize
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_Init(void)
{
    /* initialize of IIC function */
    R_IICA0_Init();

    /* initilaize of gyro function */
    R_GYRO_Init();

    /* initilaize of gyro centerPosi */
    R_GYRO_CenterPosi_Init();

    /* initialize of GrennDSP */
    R_GREEN_DSP_Init();
#if(FEEDBACK_SELECTION == HALL)
    /* initialize of hall function */
    R_HALL_Init();
#endif

    /* AD converter enable */
    R_ADON_Start();

    /* initialize of OIS drive mode setting */
    R_OIS_DrvModeSet();

    pwmDutyFixedState = OIS_DISABLE;

    CMD_REG->OISSTS = OIS_STATE_GYRO_WAKEUP_WAIT;
}

/******************************************************************************
* Function Name: R_OIS_Init_CommandReg
* Description : initialize of command registers
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_Init_CommandReg(void)
{
    uint16_t         loop;
    volatile uint8_t *buf;

    /* initial value of command register is 0 */
    buf = (volatile uint8_t *)&commandReg;

    for(loop=0; loop < 256; loop++)
    {
        buf[loop] = 0U;
    }

    /* 0x0-0xFF Set initialize data (not 0) */
    CMD_REG->OISMODE         = INIT_OISMODE;
    CMD_REG->XPWMDUTY        = INIT_XPWMDUTY;
    CMD_REG->YPWMDUTY        = INIT_YPWMDUTY;
    CMD_REG->XTARGET         = INIT_XTARGET;
    CMD_REG->YTARGET         = INIT_YTARGET;
    CMD_REG->HWVER[0]        = RUMBA_DEVICE;
    CMD_REG->HWVER[1]        = 0U;
    CMD_REG->HWVER[2]        = 0U;
    CMD_REG->HWVER[3]        = 0U;
    CMD_REG->FW_REVISION     = RUMBA_FW_REVISION;
    CMD_REG->SINFREQ         = 1U;
    CMD_REG->SQFREQ          = 1U;
    CMD_REG->DFLSSIZE_W      = 1U;

    /* 0x200-0x5FF from Data Flash */
    R_DATAFLASH_OisDataAreaLoad();

    /* for fail safe */
    if((CMD_REG->EXTCLK == 0) || (CMD_REG->PLLMULTIPLE == 0) || (CMD_REG->PLLDIVIDE == 0) ||
       (CMD_REG->I2CWL==0) || (CMD_REG->I2CWH == 0))
    {
        CMD_REG->EXTCLK      = INIT_EXTCLK;
        CMD_REG->PLLMULTIPLE = INIT_PLLMULTIPLE;
        CMD_REG->PLLDIVIDE   = INIT_PLLDIVIDE;
        CMD_REG->I2CWL       = INIT_I2CWL;
        CMD_REG->I2CWH       = INIT_I2CWH;
        CMD_REG->I2CMODE     = INIT_I2CMODE;
    }

    CMD_REG->SYSCLK = CMD_REG->EXTCLK * CMD_REG->PLLMULTIPLE / CMD_REG->PLLDIVIDE;
}

/******************************************************************************
* Function Name: R_OIS_Start
* Description : Start of OIS processing
* Arguments : timer2_used, hall sensitivity enable
* Return Value : none
******************************************************************************/
void R_OIS_Start(void)
{
    R_GREEN_DSP_Init();

    R_ANACNT_Set_PS_IS(ANACNT_PS_ENABLE);                               /* IS Power Save is enable */

#if(FEEDBACK_SELECTION == HALL)
    R_HALL_Start();
#endif

    R_OISCNT_Start();                                                   /* VCM output enable */

    R_ADON_Start();                                                     /* AD converter enable */

    R_TAU0_Channel1_Start();                                            /* timer1(1.96kHz) start */

    R_GREEN_DSP_Start();
}

/******************************************************************************
* Function Name: R_OIS_Stop
* Description : Stop of OIS processing
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_Stop(void)
{
    R_GREEN_DSP_Stop();                                                 /* GREEN_DSP stop */

    R_TAU0_Channel1_Stop();                                             /* timer1(1.96kHz) stop */

    R_ADON_Stop();                                                      /* AD converter disable */

    R_ANACNT_Set_PS_IS(ANACNT_PS_DISABLE);                              /* IS Power Save is disable */

    R_GREEN_DSP_StopWait();

    R_OISCNT_Stop();                                                    /* VCM output disable */
}

/******************************************************************************
* Function Name: R_OIS_ChangeState
* Description : Check the command registers, and decide the next state
* Arguments : none
* Return Value : next state
******************************************************************************/
static ois_state_e R_OIS_ChangeState(void)
{
    ois_state_e nextState = OIS_STATE_IDLE;

    /* priority : fwupctrl(high) > hpctrl > hcctrl > gcctrl > ggactrl > fwctrl > oisctrl > standby(low) */
    if((CMD_REG->OISCTRL & _01_OISCTRL_OISEN) == _01_OISCTRL_OISEN_ENABLE)
    {
        nextState = OIS_STATE_RUN;
    }
    if((CMD_REG->PARAMINIT & _01_PARAMINIT_INITEN) == _01_PARAMINIT_INITEN_ENABLE)
    {
        nextState = OIS_STATE_PARAM_INIT;
    }
    if((CMD_REG->OISDATAWRITE & _01_OISDATAWRITE) == _01_OISDATAWRITE_ENABLE)
    {
        nextState = OIS_STATE_OISDATA_WRITE;
    }
    if( (CMD_REG->GCCTRL & _01_GCCTRL_GSCEN) == _01_GCCTRL_GSCEN)
    {
        nextState = OIS_STATE_GYRO_CALIBRATION;
    }
    if((CMD_REG->HCCTRL & _01_HCCTRL_HCEN) == _01_HCCTRL_HCEN_ENABLE)
    {
        nextState = OIS_STATE_HALL_CALIBRATION;
    }
    if((CMD_REG->HPCTRL & _01_HPCTRL_HPEN) == _01_HPCTRL_HPEN_ENABLE)
    {
        nextState = OIS_STATE_HALL_POLARITY;
    }
    if((CMD_REG->DFIXCTRL &  _01_DFIXCTRL_DFIXEN) == _01_DFIXCTRL_DFIXEN_ENABLE)
    {
        nextState = OIS_STATE_PWM_DUTYFIXED;
    }
    if((CMD_REG->DFLSCTRL & _01_DFLSCTRL_DFLSEN) == _01_DFLSCTRL_DFLSEN_ENABLE)
    {
        R_DATAFLASH_UseDataArea_Start();
        nextState = OIS_STATE_DFLS_UPDATE;
    }
    if((CMD_REG->FWUPCTRL & _01_FWUPCTRL_FWUPEN) == _01_FWUPCTRL_FWUPEN_ENABLE)
    {
        nextState = OIS_STATE_FW_UPDATE;
    }
    return nextState;
}

/******************************************************************************
* Function Name: R_OIS_SetTestMonRegister
* Description : Setting of TEST_MON register(for debug use)
* Arguments : none
* Return Value : none
******************************************************************************/
static void R_OIS_SetTestMonRegister( void )
{
    /* Hall Monitor Setting */
    if( (CMD_REG->TESTMON_CTRL & _80_TESTMON_REQ_MASK) == _80_TESTMON_REQ  )
    {
        ANACNT_TEST_MON = (uint16_t)(CMD_REG->TESTMON_CTRL & (~_80_TESTMON_REQ));
        CMD_REG->TESTMON_CTRL &= (~_80_TESTMON_REQ);
    }
}

/******************************************************************************
* Function Name: R_OIS_main
* Description : OIS main routine
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_main(void)
{
	R_OIS_SetTestMonRegister();  /* TESTMON Register Setting Request Check */

    /* State Change */
    if( (ois_state_e)CMD_REG->OISSTS == OIS_STATE_IDLE )
    {
        CMD_REG->OISSTS = (uint8_t)R_OIS_ChangeState();
    }

    switch((ois_state_e)CMD_REG->OISSTS)
    {
        case OIS_STATE_IDLE:
            if( CMD_REG->DFLSCMD == DFLSCMD_READ ) /* UserAreaData read request check  */
            {
                R_DATAFLASH_FLS_Read();
            }
            if( CMD_REG->OISDRIVEMODE != CurrOisDrvMode )
            {
                R_OIS_DrvModeSet(); // OIS Drive Mode Change!!
            }
            break;

        case OIS_STATE_RUN:
            if(CMD_REG->OISCTRL == _01_OISCTRL_OISEN_ENABLE)
            {
            	R_GYRO_Update_TargetPosition((ois_mode_e)CMD_REG->OISMODE);
            }
            else
            {
                R_GYRO_Stop_Update_TargetPosition();
                CMD_REG->OISSTS = OIS_STATE_IDLE;
            }
            break;
#if(FEEDBACK_SELECTION == HALL)
        case OIS_STATE_HALL_POLARITY:
            if(R_HALL_Polarity() == OK)
            {
                CMD_REG->HPCTRL &= (uint8_t)(~_01_HPCTRL_HPEN);
                CMD_REG->OISSTS  = OIS_STATE_IDLE;
            }
            break;

        case OIS_STATE_HALL_CALIBRATION:
            if(R_HALL_Calibration() == OK)
            {
                CMD_REG->HCCTRL &= (uint8_t)(~_01_HCCTRL_HCEN);
                CMD_REG->OISSTS  = OIS_STATE_IDLE;
            }
            break;
#endif
        case OIS_STATE_GYRO_CALIBRATION:
            if(R_GYRO_Calibration() == OK)
            {
                CMD_REG->GCCTRL &= (uint8_t)(~_01_GCCTRL_GSCEN);
                CMD_REG->OISSTS = OIS_STATE_IDLE;
            }
            break;

        case OIS_STATE_OISDATA_WRITE:
            R_DATAFLASH_Set();
            CMD_REG->OISDATAWRITE &= (uint8_t)(~_01_OISDATAWRITE);
            CMD_REG->OISSTS = OIS_STATE_IDLE;
            break;

        case OIS_STATE_PWM_DUTYFIXED:
            if((CMD_REG->DFIXCTRL &  _01_DFIXCTRL_DFIXEN_ENABLE) == _01_DFIXCTRL_DFIXEN_ENABLE)
            {
                R_OIS_PWM_DutyFixed();
            }
            else
            {
                R_OIS_Stop_PWM_DutyFixed();
                CMD_REG->OISSTS = OIS_STATE_IDLE;
            }
            break;

        case OIS_STATE_FW_UPDATE:
            if( (CMD_REG->FWUPCTRL & 0xF0) == 0x70 )
            {
                NVIC->NVIC_ICER.LWORD = 0xFFFFFFFF;     /* disable interrupt */
                IICA->IICCTL00.BIT.SPIE0 = 1U;
                pAddress = (unsigned long *)MASKROM_ADDR_TBL;
                pROMEntryTable = (BootRomEntry)pAddress[0];
                R_OIS_EnableLVD();
                FWUPdateArg = (uint32_t)CMD_REG->FWUPCTRL;
                (pROMEntryTable)( &FWUPdateArg );  /* do not return!! */
            }
            else
            {
                CMD_REG->FWUPCTRL &= ~_01_FWUPCTRL_FWUPEN_ENABLE;
                CMD_REG->OISSTS = OIS_STATE_IDLE;
            }
            break;

        case OIS_STATE_DFLS_UPDATE:
            if( R_DATAFLASH_CtrlUseDataArea() == OK )
            {
                CMD_REG->DFLSCTRL &= (uint8_t)(~_01_DFLSCTRL_DFLSEN);
                CMD_REG->OISSTS = OIS_STATE_IDLE;
            }
            break;


        case OIS_STATE_PARAM_INIT:
            R_OIS_ParamInit();
            CMD_REG->PARAMINIT &= (uint8_t)(~(_01_PARAMINIT_INITEN | _80_PARAMINIT_ALL));
            CMD_REG->OISSTS = OIS_STATE_IDLE;
            break;

        case OIS_STATE_GYRO_WAKEUP_WAIT:
            if(R_GYRO_EnableCheck(GYRO_TURNON_WAITTIME) == OK)
            {
                CMD_REG->OISSTS  = OIS_STATE_IDLE;
            }
            break;

        default:
            break;
    }
}

/******************************************************************************
* Function Name: R_OIS_Get_FWInfo
* Description : Set the monitor data(for debug)
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_Get_FWInfo(void)
{
    if(CMD_REG->FWINFO_CTRL != 0)
    {
        NVIC->NVIC_ICER.LWORD = BIT_IICA_INTERRUPT;
        CMD_REG->X_GYRO_RAW    = xgyroraw;
        CMD_REG->Y_GYRO_RAW    = ygyroraw;
        CMD_REG->X_GYRO_CALC   = xgyrocalc;
        CMD_REG->Y_GYRO_CALC   = ygyrocalc;
        if( DMODE_LINEAR == R_OIS_GetDrvMode() )
        {
            CMD_REG->XVCM_DUTY = OISCNT->BTLISXDAC.WORD;
            CMD_REG->YVCM_DUTY = OISCNT->BTLISYDAC.WORD;
        }
        else if(DMODE_PWM == R_OIS_GetDrvMode() )
        {
            CMD_REG->XVCM_DUTY = OISCNT->VCMXDT.WORD;
            CMD_REG->YVCM_DUTY = OISCNT->VCMYDT.WORD;
        }
        CMD_REG->HAX_OUT   = R_ADON_Get_ResisAF();//R_ADON_Get_HallX();
//        CMD_REG->HAY_OUT   = R_ADON_Get_HallY();
//        CMD_REG->EXT_AD    = R_ADON_Get_AD2();
        NVIC->NVIC_ISER.LWORD = BIT_IICA_INTERRUPT;
    }
}
/******************************************************************************
* Function Name: R_OIS_PWM_DutyFixed
* Description : Process of Duty Fixd Mode
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_PWM_DutyFixed(void)
{
    int16_t xPwmDuty, yPwmDuty;

    if(pwmDutyFixedState == OIS_DISABLE)
    {
        R_ANACNT_Set_PS_IS(ANACNT_PS_ENABLE);                               /* IS Power Save is enable */
        R_HALL_Start();
        R_OISCNT_Start();           /* VCM output enable */
        R_ADON_Start();             /* AD converter enable */
        R_TAU0_Channel1_Start();
        A_SMA_Set_Duty_Init();
        pwmDutyFixedState = OIS_ENABLE;
    }
    else
    {
    	if(R_OIS_GetDrvMode() == DMODE_PIEZO )
    	{
    		A_SMA_Set_Duty();
    	}
    	else if(R_OIS_GetDrvMode() == DMODE_DUTY)
    	{
			A_SMA_Set_DutyX(CMD_REG->XAPIEZODUTY,CMD_REG->XBPIEZODUTY);
			A_SMA_Set_DutyY(CMD_REG->YAPIEZODUTY,CMD_REG->YBPIEZODUTY);
    	}
    	else
    	{
			xPwmDuty = R_OISCNT_Get_Duty((uint16_t)CMD_REG->XPWMDUTY);
			yPwmDuty = R_OISCNT_Get_Duty((uint16_t)CMD_REG->YPWMDUTY);

			R_OISCNT_Set_DutyX(xPwmDuty);
			R_OISCNT_Set_DutyY(yPwmDuty);
    	}
    }
}

/******************************************************************************
* Function Name: R_OIS_Stop_PWM_DutyFixed
* Description : Stop the Duty Fixd Mode
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_Stop_PWM_DutyFixed(void)
{
    R_OISCNT_Stop();                        /* VCM output disable */
    R_ADON_Stop();                          /* AD converter disable */
    R_ANACNT_Set_PS_IS(ANACNT_PS_DISABLE);  /* IS Power Save is disable */
    pwmDutyFixedState = OIS_DISABLE;
#if(FEEDBACK_SELECTION == RESISTOR)
    A_SMA_Set_Duty_Disable();
#endif
}

/******************************************************************************
* Function Name: R_OIS_Set_ComRegParamToFWData
* Description : Initialize the commnad register for F/W init value
* Arguments : ois_drive_mode
* Return Value : none
******************************************************************************/
static void R_OIS_Set_ComRegParamToFWData(void)
{
    if((CMD_REG->PARAMINIT & _80_PARAMINIT_ALL) == _80_PARAMINIT_ALL)
    {
        R_DATAFLASH_RestoreCmdReg();
    }
}

/******************************************************************************
* Function Name: R_OIS_ParamInit
* Description : Process of Parameter Initialize
* Arguments : none
* Return Value : none
******************************************************************************/
static void R_OIS_ParamInit(void)
{
    R_OIS_Set_ComRegParamToFWData();
    R_DATAFLASH_Set();
}

/******************************************************************************
* Function Name: R_EnableLVD
* Description : Set LVD Enable
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_EnableLVD(void)
{
    if( (FLC->FLCUSROPT2.LWORD&LVDSTS_MASK) == LVDSTS_DISABLE )
    {
        FLC->FLCUSROPT2.LWORD &= LVD_THRESHOLD_MASK;
        FLC->FLCUSROPT2.LWORD |= LVD_THRESHOLD_2p4V;
    }
    FLC->FLCUSROPT2.LWORD &= LVDSETTING_ENABLE;
}

/******************************************************************************
* Function Name: R_DisableLVD
* Description : Set LVD Disable
* Arguments : none
* Return Value : none
******************************************************************************/
void R_OIS_DisableLVD(void)
{
    FLC->FLCUSROPT2.LWORD |= LVDSETTING_DISABLE;
}

