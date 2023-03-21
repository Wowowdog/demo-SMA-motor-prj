/*
 * sma_r.c
 *
 *  Created on: Nov 26, 2019
 *      Author: P. Huang
 */
#include "RUMBA_sfr.h"
#include "sma_r.h"
#include "r_userdefine.h"
#include "r_ois.h"
#include "r_adon.h"
#include "r_gyro.h"

/******************************************************************************
Macro definitions
******************************************************************************/
#define R_ADSCAN_COUNT           (0U)
#define R_ADSCAN_DIVISION		 (0U)
#define R_DUTYSET_COUNT			 (3)

/* Private macro -------------------------------------------------------------*/

extern int32_t init_inFF[];
extern int32_t  init_inXp_b1[];
extern int32_t init_inYp_b1[];
extern int32_t  init_PstnDst[];
extern int32_t init_Est[];
extern int32_t init_adc[];
int32_t i = 0;
int8_t flag = 0;
int32_t POut,IOut,DOut,KcOut;
#if(FEEDBACK_SELECTION == RESISTOR)
extern const pwrparam_t initpwrparam_t;
#endif
extern const AXSV initAXSV_t;

uint8_t InvR,Pwrfnpt,InvAmp,InvAmp2Amp,Ext2Pwr,n;
uint8_t stsRFb;
uint16_t dutyXA,dutyXB,dutyYA,dutyYB;
uint32_t A1Coef;//A1Coef is Vm>>1
int32_t dRCoef;
uint8_t adcRequestFlag,clm_count;
uint32_t rAdScanSum;
uint16_t rAdScanCount,rDutySetCount;
int32_t InvK,InvKp,InvPmean;
int32_t InvVds;
/*------------vds----------*/
int32_t Vds_X_B,Vds_X_A,Vds_Y_B,Vds_Y_A;
sma_state_e setDutySeq;
#if(FEEDBACK_SELECTION == RESISTOR)
pwrparam_t PwrParam;
#endif
AXSV _Y;
AXSV _X;
#if(FEEDBACK_SELECTION == RESISTOR)
int32_t A_SMA_Get_DrX(void);
int32_t A_SMA_Get_DrY(void);
#endif
uint32_t A_SMA_GetVddfCoef(ois_axis_select_e axis);
uint32_t A_SMA_GetVdiff(ois_axis_select_e axis, uint16_t ADCr);
int32_t A_SMA_GetDR(uint32_t tempconst,uint16_t Vdiff2,uint16_t Vdiff1,int32_t Vds_2, int32_t Vds_1);
uint16_t A_SMA_GetPrividedPower(uint32_t PCoef1, uint32_t PCoef2 ,uint32_t Vdiff, uint32_t InvCoef, int8_t _S);
int32_t A_SMA_MappingFunction_Cubic(int32_t _MP3, int32_t _MP2, int32_t _MP1,int32_t _dR, int32_t _MP0, int32_t InvExtendCoef, int32_t ExtendEst);
int32_t A_SMA_Set_Target(int16_t minstrk, int16_t maxstrk, uint16_t target);
int32_t A_SMA_Power_Add(uint16_t mean,int32_t adder, int32_t limit);
int32_t A_SMA_FFControl(int32_t _inA10dD, int32_t _inB7dD, int32_t _inC7dD, int32_t _inYp_b1, int32_t _PstnDstExt, int32_t _inXp_b1);
int32_t A_SMA_DutyCal(int32_t NumeratorExt , int32_t DenominatorExt, int32_t NumberOfExtForDutyR);
int32_t Input_Ext(int32_t Input, int32_t PowerOfExt);
int32_t A_SMA_Linear(int32_t Variable, int32_t Real_A, int32_t Real_B, int8_t Inv);
void A_SMA_TC_Init_Variable(void);
#if(FEEDBACK_SELECTION == RESISTOR)
void A_SMA_MosDis(void);
void A_SMA_MosEn(void);
void A_SMA_MosEnY(void);
void A_SMA_MosEnX(void);
static uint32_t A_SMA_GetAmpGain(void);
static uint32_t A_SMA_Get2ndAmpGain(void);
#endif

#if(FEEDBACK_SELECTION == RESISTOR)
//********************************************************************//
//							getXDR()
//	CLM
//	Parameters:
//*****************************************//
int32_t A_SMA_Get_DrX(void)
{
	int32_t tempDR;
	tempDR = A_SMA_GetDR(PwrParam.ParamRsG1G2s12_X,A_SMA_GetVdiff(OIS_AXIS_X_P,CMD_REG->RXB_OUT),A_SMA_GetVdiff(OIS_AXIS_X_N,CMD_REG->RXA_OUT),
			Vds_X_B,
			Vds_X_A);
	return tempDR;
}
//********************************************************************//
//							getYDR()
//	CLM
//	Parameters:
//*****************************************//
int32_t A_SMA_Get_DrY(void)
{
	int32_t tempDR;
	tempDR = A_SMA_GetDR(PwrParam.ParamRsG1G2s12_Y,A_SMA_GetVdiff(OIS_AXIS_Y_P,CMD_REG->RYB_OUT),A_SMA_GetVdiff(OIS_AXIS_Y_N,CMD_REG->RYA_OUT),
			Vds_Y_B,
			Vds_Y_A);
	return tempDR;
}
#endif
/******************************************************************************
* Function Name: A_SMA_Init_Variable
* Description : initialize of variable
* Arguments : none
* Return Value : none
******************************************************************************/
void A_SMA_Init_Variable(void)
{

	InvR = CMD_REG->EXT1_1+((CMD_REG->EXT2)<<1);
	Pwrfnpt = (uint8_t)CMD_REG->EXT4-CMD_REG->EXT3;
	InvVds = 8 - CMD_REG->EXT11;
	stsRFb = OIS_AXIS_Y;
	setDutySeq = SMA_HW_YA;

	n = 0;
#if(FEEDBACK_SELECTION == RESISTOR)
	PwrParam = initpwrparam_t;
#endif
	_Y = initAXSV_t;
	_X = initAXSV_t;
	/*FF*/
	_X._in2T2_txK = _Y._in2T2_txK = ((CMD_REG->T2<<1)-CycleTime)*CMD_REG->K;
	_X._in2T1pT  = _Y._in2T1pT  = (CMD_REG->T1<<1)+CycleTime;
	_X._inT_2T1   = _Y._inT_2T1   = CycleTime-(CMD_REG->T1<<1);
	_X._in2T2pTxK= _Y._in2T2pTxK= ((CMD_REG->T2<<1)+CycleTime)*CMD_REG->K;
	_X._inA10dD  = _Y._inA10dD  =	(_X._in2T2_txK<<10)/_X._in2T2pTxK;
	_X._inB7dD   = _Y._inB7dD   =	(_X._in2T1pT<<7)/_X._in2T2pTxK;
	_X._inC7dD   = _Y._inC7dD	=   (_X._inT_2T1<<7)/_X._in2T2pTxK;
	/*TC*/
	/*KP*/
	/*K*/
	/*Mean Power*/

//	A_SMA_TC_Init_Variable();



}
/******************************************************************************
* Function Name: A_SMA_TC_Init_Variable
* Description : initialize of variable
* Arguments : none
* Return Value : none
******************************************************************************/
void A_SMA_TC_Init_Variable(void)
{
	InvK = CMD_REG->EXT0-CMD_REG->EXT9;
	InvKp = 8 - CMD_REG->EXT7;
	InvPmean = CMD_REG->EXT5- CMD_REG->EXT8;

	CMD_REG->K = A_SMA_Linear(CMD_REG->T, CMD_REG->K_A, CMD_REG->K_B, InvK);
	CMD_REG->YKP = A_SMA_Linear(CMD_REG->T, CMD_REG->KP_A, CMD_REG->KP_B, InvKp);
	CMD_REG->XKP = A_SMA_Linear(CMD_REG->T, CMD_REG->KP_A, CMD_REG->KP_B, InvKp);
	CMD_REG->MEANPOWERYA = A_SMA_Linear(CMD_REG->T, CMD_REG->PMEAN_N_A, CMD_REG->PMEAN_N_B, InvPmean);
	CMD_REG->MEANPOWERYB = A_SMA_Linear(CMD_REG->T, CMD_REG->PMEAN_P_A, CMD_REG->PMEAN_P_B, InvPmean);
	CMD_REG->MEANPOWERXA = A_SMA_Linear(CMD_REG->T, CMD_REG->PMEAN_N_A, CMD_REG->PMEAN_N_B, InvPmean);
	CMD_REG->MEANPOWERXB = A_SMA_Linear(CMD_REG->T, CMD_REG->PMEAN_P_A, CMD_REG->PMEAN_P_B, InvPmean);

	_Y._inPwrMean2 = CMD_REG->MEANPOWERYB+CMD_REG->MEANPOWERYA;

	_X._inPwrMean2 = CMD_REG->MEANPOWERXB+CMD_REG->MEANPOWERXA;

}
#if(FEEDBACK_SELECTION == RESISTOR)
/******************************************************************************
* Function Name: A_SMA_GetVddfCoef
* Description : Get coef1 value
* Arguments : axis
* Return Value : Coef1 value after multiplying 2^Ext12
******************************************************************************/
uint32_t A_SMA_GetVddfCoef(ois_axis_select_e axis)
{
    uint32_t tempRCoef;

    switch( axis )
    {
        case (OIS_AXIS_X_N):
        case (OIS_AXIS_X_P):
        		tempRCoef = PwrParam.ParamG2OfstG2Vm_X;
        	break;
        case (OIS_AXIS_Y_N):
        case (OIS_AXIS_Y_P):
				tempRCoef = PwrParam.ParamG2OfstG2Vm_Y;
        	break;
    }
    return( tempRCoef );
}
/******************************************************************************
* Function Name: A_SMA_GetVdiff
* Description : Get Vdiff value
* Arguments : axis
* Return Value : Vdiff value multiply 2^Ext12
******************************************************************************/
uint32_t A_SMA_GetVdiff(ois_axis_select_e axis, uint16_t ADCr)
{
	uint32_t tempVdiff;
		tempVdiff = A_SMA_GetVddfCoef(axis) - ADCr;
	return tempVdiff;
}
/******************************************************************************
* Function Name: A_SMA_GetDR
* Description : Get Delta R value
* Arguments : axis
* tempconst = old : 4.2*2^(ext14+ext2)=> new : Vp(Vm * Rs) = Vm*Rs*2^(ext14+ext2)
* Return Value : Delta R value multiply 2^ext2
******************************************************************************/
int32_t A_SMA_GetDR(uint32_t tempconst,uint16_t Vdiff2,uint16_t Vdiff1,int32_t Vds_2, int32_t Vds_1)
{
	int32_t dR,temp_VM_Vds2,temp_VM_Vds1;

//	dR = ((((tempconst* (Vdiff1-Vdiff2))/(Vdiff2*Vdiff1))*(255-CMD_REG->MOSV))>>(8));
	temp_VM_Vds2 = (255 - Vds_2);
	temp_VM_Vds1 = (255 - Vds_1);

//	dR = (((tempconst* ((temp_VM_Vds2/Vdiff2)-(temp_VM_Vds1/Vdiff1)))>>8));
	dR = ((tempconst*(((temp_VM_Vds2*Vdiff1)-(temp_VM_Vds1*Vdiff2))/(Vdiff2*Vdiff1)))>>8);

	return dR;//uint is ohm/2^Ex2
}
#endif
/******************************************************************************
* Function Name: A_SMA_GetPower
* Description : Get the power the system could provide
* Arguments : axis
* PCoef1 = Vm * 1000(mV/V) / Rs (Ohm) * 2^(ext5-ext12)
* PCoef2 = 2^(ext5-ext12*2)*1000/Rs (Ohm)
* Return Value : Delta R value multiply 2^Ext5
* https://embeddedgurus.com/stack-overflow/2009/06/division-of-integers-by-constants/
******************************************************************************/
uint16_t A_SMA_GetPrividedPower(uint32_t PCoef1, uint32_t PCoef2 ,uint32_t Vdiff, uint32_t InvCoef, int8_t _S)
{
	uint16_t tempPower,tempP1,tempP2;
	uint8_t tempShift;
	tempShift = CMD_REG->EXT5 - CMD_REG->EXT10;
	if(tempShift > 0)
	{
		tempP1 = ((uint32_t)Vdiff*(uint32_t)(InvCoef))>>_S;//----Vdiff/InvCoef <<16
		tempP2 = (uint32_t)(uint32_t)((((PCoef2*tempP1)>>16)*tempP1)>>16)<<tempShift;
		tempP1 = (uint32_t)((PCoef1*tempP1)<<tempShift)>>16;
		tempPower = tempP1-tempP2;
	}
	else
	{
		tempP1 = ((uint32_t)Vdiff*(uint32_t)(InvCoef))>>_S;//----Vdiff/InvCoef <<16
		tempP2 = (uint32_t)(uint32_t)((((PCoef2*tempP1)>>16)*tempP1)>>16)>>tempShift;
		tempP1 = (uint32_t)((PCoef1*tempP1)>>tempShift)>>16;
		tempPower = tempP1-tempP2;
	}


	return tempPower;
}

//******************************************************************************************//
//									A_SMA_MappingFunction_Cubic()
//	Parameters:
//	DeltaR(mR) 							: the difference of resistance
//	Square_Coef_Extend(um)				: The coefficient of square
//	One_Order_Coef_Extend(um)			: The coefficient of one order
//	Constant_Coef_Extend(um)			: the coefficient of constant

//	Output
//	out_Estmation(um)					: extend the 2^ExtendForDeltaResistance
//*****************************************//
int32_t A_SMA_MappingFunction_Cubic(int32_t _MP3, int32_t _MP2, int32_t _MP1,int32_t _dR, int32_t _MP0, int32_t InvExtendCoef, int32_t ExtendEst)
{
	int32_t out_Est,temp;
	//constant
	if((CMD_REG->EXT3-CMD_REG->EXT1_0)>0)
		out_Est = ((_MP0<<(CMD_REG->EXT2*3)))<<(CMD_REG->EXT3-CMD_REG->EXT1_0);
	else
		out_Est = ((_MP0<<(CMD_REG->EXT2*3)))>>(CMD_REG->EXT1_0-CMD_REG->EXT3);
	//1st
	if((CMD_REG->EXT3-CMD_REG->EXT1_1)>0)
		out_Est += (((_MP1*_dR)<<(CMD_REG->EXT2<<1))<<(CMD_REG->EXT3-CMD_REG->EXT1_1));
	else
		out_Est += (((_MP1*_dR)<<(CMD_REG->EXT2<<1))>>(CMD_REG->EXT1_1-CMD_REG->EXT3));
	//3rd+2nd
	temp = (_MP3*_dR*_dR);
	if((CMD_REG->EXT1_3-CMD_REG->EXT1_2) > 0)
		temp = (temp>>(CMD_REG->EXT1_3-CMD_REG->EXT1_2))*_dR;//3rd
	else
		temp = (temp<<(CMD_REG->EXT1_3-CMD_REG->EXT1_2))*_dR;//3rd

	temp += (_MP2*_dR*_dR)<<(CMD_REG->EXT2);//2nd

	if((CMD_REG->EXT1_2-CMD_REG->EXT3)>0)
	{
		temp = (temp>>(CMD_REG->EXT1_2-CMD_REG->EXT3));
	}
	else
	{
		temp = (temp<<(CMD_REG->EXT3-CMD_REG->EXT1_2));
	}

	out_Est += temp;
	return out_Est;
}
/*********************************************************************
  * @brief  Set setpoint via I2C
  * @param  None
  * @retval None
********************************************************************/
//int32_t A_SMA_Set_Target(pAXSV inAxis)
int32_t A_SMA_Set_Target(int16_t minstrk, int16_t maxstrk, uint16_t target)
{
	int32_t result;
	result = (int32_t)((int32_t)((minstrk)<<CMD_REG->EXT3) +
			(((maxstrk - minstrk)<<CMD_REG->EXT3)*(int32_t)target>>12));
	return result;
}
//********************************************************************//
//							A_SMA_Power_Add()
//	extend the command into PID Controller function
//	Parameters:
//	PowerMeanInput(unit is the TBD) 		:To be extended 1. Setpoint 2. estimated position
//	PowerInputFromController(none)			:The number of power for Extension
//  Direction(bool)							:the value is changed by direction, up is 1 down is 0
//
//	output
//	PowerTBAdjust(none)			:Recent input after extending
//*****************************************//
int32_t A_SMA_Power_Add(uint16_t mean,int32_t adder, int32_t limit)
{
	int32_t PowerAdPos;
	PowerAdPos = (int32_t)(mean + adder);
		if(PowerAdPos >= limit)
		{
			return limit;
		}
		else if(PowerAdPos <= 0)
		{
			return 0;
		}
		else
		{
			return PowerAdPos;
		}

}
//******************************************************************************************//
//									A_SMA_FFControl
//	Parameters:
//	DoubleKTTwo						: 2KT2	(mm x msec/mW) extend 100
//	DoubleTOne						: 2T1	msec
//	T								: T		msec	extend 100
//	K								: K
//  PowerFFControllerOnePointBefore : Y(t-1), Cycle time at t-1 milliseconds(mW)
//  DesiredPosition 				: X(t), set point at t milliseconds
//  DesiredPositionOnePointBefore	: X(t-1), set point at t-1 milliseconds.
//	CommonDenominator				: for the equation
//  ExtendForPower					: Extend for PowerFromController

//	Output
//	PowerFFControlExtended(mW)			: Y(t),unit is mW extend : _____  the output from feed forward controller
//*****************************************//
int32_t A_SMA_FFControl(int32_t _inA10dD, int32_t _inB7dD, int32_t _inC7dD, int32_t _inYp_b1, int32_t _PstnDstExt, int32_t _inXp_b1)
{
	int32_t result;
	int8_t turn;
	turn = CMD_REG->FF;
	   switch(turn)
	   {
	   case 0:
			   {result = 0;
			   	   	   	   }
			   break;
	   case 1:
			   {result = ( ((_inA10dD*_inYp_b1)>>10) + ((_inB7dD*_PstnDstExt)>>7) + ((_inC7dD *_inXp_b1)>>7) );

			   	   }
			   break;
	   }
	return result;//EX5
}
//******************************************************************************************//
//									A_SMA_DutyCal
//	Parameters:
//NumeratorExt : the power should provide
//DenominatorExt : the power that system has(PIEZOXTIME1 or PIEZOYTIME1)
//NumberOfExtForDutyR : the resolution of the system could provide

//	Output: the duty value of the channel chosen
//*****************************************//
int32_t A_SMA_DutyCal(int32_t NumeratorExt , int32_t DenominatorExt, int32_t NumberOfExtForDutyR)
{
	int32_t DutyRatioExt;
		//extend Time Period
#if(FEEDBACK_SELECTION == HALL && PRODUCT_SELECTION == ROLL_OIS)
		DutyRatioExt = ((NumeratorExt)*33157)>>13;//original
		DutyRatioExt = (DutyRatioExt*NumberOfExtForDutyR)>>16;
#elif(FEEDBACK_SELECTION == RESISTOR && PRODUCT_SELECTION == ROLL_OIS)
//		DutyRatioExt = ((NumeratorExt)*0xBA2F)>>13;//original
//		DutyRatioExt = (DutyRatioExt*NumberOfExtForDutyR)>>16;
		DutyRatioExt = (TOTALTIMEPERIOD*NumeratorExt);
		DutyRatioExt -= (HOLDINGRATIO*DenominatorExt);
		DutyRatioExt = DutyRatioExt<<6;
		DutyRatioExt /= (TOTALTIMEPERIOD*DenominatorExt);
		DutyRatioExt = DutyRatioExt<<5;
//		DutyRatioExt = (((TOTALTIMEPERIOD*NumeratorExt)<<11)-((HOLDINGRATIO*DenominatorExt)<<11))/(TOTALTIMEPERIOD*DenominatorExt);
//		DutyRatioExt = ((100*NumeratorExt-HOLDINGRATIO*DenominatorExt)<<10)/(RESTRATIO*DenominatorExt);
//		DutyRatioExt = ((((100*NumeratorExt)<<10)/DenominatorExt)-((HOLDINGRATIO)<<10))/RESTRATIO;
//		DutyRatioExt = (NumeratorExt*NumberOfExtForDutyR)/DenominatorExt;//original
#else
		DutyRatioExt = (NumeratorExt*NumberOfExtForDutyR)/DenominatorExt;//original
#endif
		if (DutyRatioExt >= (NumberOfExtForDutyR))//LmtDuty
			DutyRatioExt = (NumberOfExtForDutyR);//LmtDuty
		else if(DutyRatioExt <= 0)//MinDuty
			DutyRatioExt = 0;//MinDuty

	return DutyRatioExt;
}
//********************************************************************//
//							Input_Ext()
//	extend the command into PID Controller function
//	Parameters:
//	Input(unit is the TBD) 		:To be extended 1. Setpoint 2. estimated position
//	PowerOfExtend(none)			:The number of power for Extension
//
//	output
//	ExtendedOuput(none)			:Recent input after extending
//*****************************************//
int32_t Input_Ext(int32_t Input , int32_t PowerOfExt)
{
	int32_t ExtendedOuput;
	if(PowerOfExt>0)
		ExtendedOuput = (int32_t)(Input<<PowerOfExt);
	else if(PowerOfExt<0)
		ExtendedOuput = (int32_t)(Input>>(abs(PowerOfExt)));
	return ExtendedOuput;
}
//****************************************//
//									A_SMA_LINEAR
//	Parameters:
// input : temperature(degree)

//	Output: the duty value of X_A channel chosen
//*****************************************//
int32_t A_SMA_Linear(int32_t Variable, int32_t Real_A, int32_t Real_B, int8_t Inv)
{
	//disable system
	//compensation doing
	int32_t temp = 0;
	temp = Real_A * Variable + Real_B;
	return Input_Ext(temp,Inv);

	//enable system
}
//****************************************//
//									R_SMA_Set_DutyXA
//	Parameters:
// input : duty value

//	Output: the duty value of X_A channel chosen
//*****************************************//
void A_SMA_Set_DutyX(uint16_t _A, uint16_t _B)
{
	if( R_OIS_GetDrvMode() == DMODE_PIEZO )
	{
	OISCNT->PIEZOXTIME3.WORD = 1;//0x007F;//Xb-start
	OISCNT->PIEZOXTIME4.WORD = (_B);//0x007F;//Xb-end
	OISCNT->PIEZOXTIME2.WORD = 1+(_B);//Xa-start
	OISCNT->PIEZOXTIME5.WORD = (_A);//Xa-end
	}
}
//****************************************//
//									A_SMA_Set_DutyY
//	Parameters:
// input : duty value

//	Output: the duty value of Y_A channel chosen
//*****************************************//
void A_SMA_Set_DutyY(uint16_t _A, uint16_t _B)
{
	if( R_OIS_GetDrvMode() == DMODE_PIEZO )
	{OISCNT->PIEZOYTIME3.WORD = 1;//0x007F;//Yb-start
	OISCNT->PIEZOYTIME4.WORD = (_B);//0x007F;//Yb-end
	OISCNT->PIEZOYTIME2.WORD = 1+(_B);//Ya-start
	OISCNT->PIEZOYTIME5.WORD = (_A);}//Ya-end
}
#if(FEEDBACK_SELECTION == RESISTOR)
void A_SMA_MosDis(void)
{
	OISCNT->OISMODE.BIT.CNT_ISX_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISX_LSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_LSOFF = 1;
}
//*****************************************//
//									A_SMA_MosEn
//	Parameters:
// input :
//	Description : This function set the switch on for all channels
//	Output:
//*****************************************//
void A_SMA_MosEn(void)
{
	OISCNT->OISMODE.BIT.CNT_ISX_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISX_LSOFF = 0;
	OISCNT->OISMODE.BIT.CNT_ISY_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_LSOFF = 0;
}
//******************************************//
//									A_SMA_MosEnY
//	Parameters:
// input :
//	Description : This function set the switch on for y channel
//	Output:
//*****************************************//
void A_SMA_MosEnY(void)
{
	OISCNT->OISMODE.BIT.CNT_ISX_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISX_LSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_LSOFF = 0;
}
//*****************************************//
//									A_SMA_MosEnX
//	Parameters:
// input :
//	Description : This function set the switch on for x channel
//	Output:
//*****************************************//
void A_SMA_MosEnX(void)
{
	OISCNT->OISMODE.BIT.CNT_ISX_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISX_LSOFF = 0;
	OISCNT->OISMODE.BIT.CNT_ISY_HSOFF = 1;
	OISCNT->OISMODE.BIT.CNT_ISY_LSOFF = 1;
}
#endif
void A_SMA_Set_Duty_Init(void)
{
	setDutySeq = SMA_HW_YA;
	clm_count = 0;
#if(FEEDBACK_SELECTION == RESISTOR)
	A_SMA_MosEn();
	adcRequestFlag = 0U;
#endif
}
//******************************************//
//									A_SMA_Set_Duty
//	Parameters:
// input :
//	Description : This function set the power of all channels
//	Output:
//*****************************************//
void A_SMA_Set_Duty(void)
{
#if(FEEDBACK_SELECTION == HALL)
#if(CHANNEL_SELECTION == CHANNEL_AF)
		CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
#elif(CHANNEL_SELECTION == CHANNEL_Y)
		CMD_REG->RYA_OUT = R_ADON_Get_HallY();
#endif
	CMD_REG->RYB_OUT = 2048;
	#if(PRODUCT_SELECTION == ROLL_OIS | PRODUCT_SELECTION == PRISM)
			CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);/*35.8us*/
			CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);/*35.8us*/
			A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY<<1),(CMD_REG->YBPIEZODUTY<<1));//pwm_yb
			CMD_REG->FBY = CMD_REG->RYB_OUT-CMD_REG->RYA_OUT;
	#elif(PRODUCT_SELECTION == OIS)
		switch(setDutySeq)
		{
			case SMA_HW_YA:
				A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY<<1),0);//pwm_ya
				setDutySeq = SMA_GET_ADC_YA;
				break;
			case SMA_GET_ADC_YA:
				A_SMA_Set_DutyY(0,(CMD_REG->YBPIEZODUTY<<1));//pwm_yb
				setDutySeq = SMA_GET_ADC_YB;
				break;
			case SMA_GET_ADC_YB:
				A_SMA_Set_DutyX((CMD_REG->XAPIEZODUTY<<2),0);
				setDutySeq = SMA_GET_ADC_XA;
				break;
			case SMA_GET_ADC_XA:
				A_SMA_Set_DutyX(0,(CMD_REG->XAPIEZODUTY<<2));
				setDutySeq = SMA_HW_YA;
				break;
		}


	#endif

#elif(FEEDBACK_SELECTION == RESISTOR)
	#if((PRODUCT_SELECTION == ROLL_OIS | PRODUCT_SELECTION == PRISM))
		if(adcRequestFlag == 1U)
		{
			adcRequestFlag = 0;
				switch(setDutySeq)
				{
					case SMA_HW_YA:
						PORT->GPIO0DT.BIT.PTD04 = 1;
						A_SMA_Set_DutyY(0x3fe,0);//A
						PORT->GPIO0DT.BIT.PTD04 = 0;
						setDutySeq = SMA_GET_ADC_YB;
						break;
					case SMA_GET_ADC_YB:
						PORT->GPIO0DT.BIT.PTD07 = 1;
						#if (CHANNEL_SELECTION == CHANNEL_AF)
							CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
						#elif (CHANNEL_SELECTION == CHANNEL_Y)
							CMD_REG->RYA_OUT = R_ADON_Get_HallY();
						#elif (CHANNEL_SELECTION == CHANNEL_XY)
							CMD_REG->RYA_OUT = R_ADON_Get_HallY();
						#endif
						PORT->GPIO0DT.BIT.PTD07 = 0;
						A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),(CMD_REG->YBPIEZODUTY));//y_pwm
						Vds_Y_A = ((CMD_REG->VDS_A * CMD_REG->RYA_OUT)>>(CMD_REG->EXT11-8)) + ((CMD_REG->VDS_B)>>(CMD_REG->EXT12-8));
						if(Vds_Y_A <= 0)
							Vds_Y_A = 0;
						CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);/*35.8us*/

						setDutySeq = SMA_GET_ADC_XA;
						break;
					case SMA_GET_ADC_XA:
						A_SMA_Set_DutyY(0,0x3fe);//B
						setDutySeq = SMA_GET_DUTY_X;
						break;
					case SMA_GET_DUTY_X:
						PORT->GPIO0DT.BIT.PTD07 = 1;
						#if (CHANNEL_SELECTION == CHANNEL_AF)
							CMD_REG->RYB_OUT = R_ADON_Get_ResisAF();
						#elif (CHANNEL_SELECTION == CHANNEL_Y)
							CMD_REG->RYB_OUT = R_ADON_Get_HallY();
						#elif (CHANNEL_SELECTION == CHANNEL_XY)
							CMD_REG->RYB_OUT = R_ADON_Get_HallX();
						#endif
						PORT->GPIO0DT.BIT.PTD07 = 0;
						A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),(CMD_REG->YBPIEZODUTY));//y_pwm
						Vds_Y_B = ((CMD_REG->VDS_A * CMD_REG->RYB_OUT)>>(CMD_REG->EXT11-8)) + ((CMD_REG->VDS_B)>>(CMD_REG->EXT12-8));
						if(Vds_Y_B <= 0)
							Vds_Y_B = 0;
						CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);/*35.8us*/

						setDutySeq = SMA_SET_DUTY_XB;
						break;
					case SMA_SET_DUTY_XB:
						if(rDutySetCount == 0)
						{
							A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),(CMD_REG->YBPIEZODUTY));//ya_pwm
							CMD_REG->FBY = A_SMA_Get_DrY();
							rDutySetCount++;
							setDutySeq = SMA_SET_DUTY_XB;
						}
						else if(rDutySetCount <= R_DUTYSET_COUNT)
						{
							rDutySetCount++;
							setDutySeq = SMA_SET_DUTY_XB;
						}
						else
						{
							rDutySetCount = 0;
							setDutySeq = SMA_HW_YA;
						}
						break;
				}
		}
    #elif((CHANNEL_SELECTION == CHANNEL_XY) && (PRODUCT_SELECTION == ROLL_OIS | PRODUCT_SELECTION == PRISM))
		CMD_REG->RYA_OUT = R_ADON_Get_HallY();
		CMD_REG->RYB_OUT = R_ADON_Get_HallX();
		CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);/*35.8us*/
		CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);/*35.8us*/
		A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY<<1),(CMD_REG->YBPIEZODUTY<<1));//pwm_yb
	#endif
#endif
}
//******************************************************************************************//
//									A_SMA_OutputTargetPosition
//	Parameters:
// input :
//	Description : This function get the target of the unit of value
//	Output:
//*****************************************//
void A_SMA_OutputTargetPosition(void)
{
	PORT->GPIO0DT.BIT.PTD04 = 1;
#if(FEEDBACK_SELECTION == HALL)
	#if(PRODUCT_SELECTION == ROLL_OIS || PRODUCT_SELECTION == PRISM)
#if(CHANNEL_SELECTION == CHANNEL_AF)
		CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
#elif(CHANNEL_SELECTION == CHANNEL_Y)
		CMD_REG->RYA_OUT = R_ADON_Get_HallY();
#endif
		CMD_REG->RYB_OUT = 2048;
					CMD_REG->FBY = (CMD_REG->RYB_OUT-CMD_REG->RYA_OUT);
					_Y._outPstnEst = A_SMA_MappingFunction_Cubic
							(CMD_REG->YMP_3, CMD_REG->YMP_2, CMD_REG->YMP_1, CMD_REG->FBY,CMD_REG->YMP_0, InvR, CMD_REG->EXT3);//4x
					/*-----------Estimation and setpoint from command extension--------------*/
					_Y._PstnDstExt = _Y._PstnDst<<Pwrfnpt;
					CMD_REG->ERRY = (int16_t)(_Y._PstnDst - _Y._outPstnEst);//for PID  2^(ex3)
					_Y._inKIErr += CMD_REG->ERRY;
					_Y._inFF = (A_SMA_FFControl(_Y._inA10dD, _Y._inB7dD, _Y._inC7dD, _Y._inYp_b1, _Y._PstnDstExt, _Y._inXp_b1))
							+ (((int32_t)CMD_REG->YKP*CMD_REG->ERRY)>>8)
							+ (((int32_t)CMD_REG->YKI*_Y._inKIErr)>>8)
							+ (((int32_t)CMD_REG->YKD*(CMD_REG->ERRY- _Y._inKDErr))>>8);
//							+ A_SMA_PIDControl(&_Y ,(int32_t)CMD_REG->YKP,  (int32_t)CMD_REG->ERRY,  (int32_t)CMD_REG->YKD, (int32_t)CMD_REG->YKI,  _Y._inKDErr,  _Y._inKIErr ,  (CMD_REG->YAPIEZOPWR),  (int32_t)CMD_REG->YKC,  2147483648,  -2147483648);


					_Y._inKDErr = CMD_REG->ERRY;
					 _Y._inYp_b1 = _Y._inFF;
					 _Y._inXp_b1 = _Y._PstnDstExt;
					CMD_REG->YAFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYA,-(_Y._inFF), CMD_REG->YAPIEZOPWR);/*8.1us*/
					CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);

					CMD_REG->YBFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYB,_Y._inFF, CMD_REG->YBPIEZOPWR);
					CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);
					A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY<<1),(CMD_REG->YBPIEZODUTY<<1));//pwm_yb
		setDutySeq = SMA_HW_YA;
PORT->GPIO0DT.BIT.PTD04 = 0;
	#elif(PRODUCT_SELECTION == OIS)
		if(adcRequestFlag == 1U)
			{
			switch(setDutySeq)
					{
						case SMA_HW_YA:
							#if(CHANNEL_SELECTION == CHANNEL_AF)
									CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
							#elif(CHANNEL_SELECTION == CHANNEL_Y)
									CMD_REG->RYA_OUT = R_ADON_Get_HallY();
							#endif
							CMD_REG->RYB_OUT = 2048;
							CMD_REG->FBY = (CMD_REG->RYB_OUT-CMD_REG->RYA_OUT);
							_Y._outPstnEst = A_SMA_MappingFunction_Cubic
									(CMD_REG->YMP_3, CMD_REG->YMP_2, CMD_REG->YMP_1,CMD_REG->FBY,CMD_REG->ParamYMP_0, InvR, CMD_REG->EXT3);//4x
							/*-----------Estimation and setpoint from command extension--------------*/
							_Y._PstnDstExt = _Y._PstnDst<<Pwrfnpt;
							CMD_REG->ERRY = (int16_t)(_Y._PstnDst - _Y._outPstnEst);//for PID  2^(ex3)

							_Y._inKIErr += CMD_REG->ERRY;
							_Y._inFF = (A_SMA_FFControl(_Y._inA10dD, _Y._inB7dD, _Y._inC7dD, _Y._inYp_b1, _Y._PstnDstExt, _Y._inXp_b1))
									+ (((int32_t)CMD_REG->YKP*CMD_REG->ERRY)>>8)
									+ (((int32_t)CMD_REG->YKI*_Y._inKIErr)>>8)
									+ (((int32_t)CMD_REG->YKD*(CMD_REG->ERRY- _Y._inKDErr))>>8);

							_Y._inKDErr = CMD_REG->ERRY;
							_Y._inYp_b1 = _Y._inFF;
							 _Y._inXp_b1 = _Y._PstnDstExt;
							CMD_REG->YBFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYB,_Y._inFF, CMD_REG->YAPIEZOPWR);
							CMD_REG->YAFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYA,-(_Y._inFF), CMD_REG->YBPIEZOPWR);/*8.1us*/
						/*35.8us*/	CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);
							CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);
							A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),0);
							A_SMA_Set_DutyY(0,(CMD_REG->YBPIEZODUTY));//yb_pwm
							setDutySeq = SMA_GET_ADC_YA;
							break;
						case SMA_GET_ADC_YA:
							#if(CHANNEL_SELECTION == CHANNEL_AF)
									CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
							#elif(CHANNEL_SELECTION == CHANNEL_Y)
									CMD_REG->RYA_OUT = R_ADON_Get_HallY();
							#endif
							CMD_REG->RXB_OUT = 2048;
							CMD_REG->FBX = (CMD_REG->RXB_OUT-CMD_REG->RXA_OUT);
							_X._outPstnEst = A_SMA_MappingFunction_Cubic
									(CMD_REG->XMP_3, CMD_REG->XMP_2, CMD_REG->XMP_1, CMD_REG->FBX ,CMD_REG->XMP_0, InvR, CMD_REG->EXT3);//4x
							/*-----------Estimation and setpoint from command extension--------------*/
							_X._PstnDstExt = _X._PstnDst<<Pwrfnpt;
							CMD_REG->ERRX = (int16_t)(_X._PstnDst - _X._outPstnEst);//for PID  2^(ex3)
							_X._inKIErr += CMD_REG->ERRX;
							_X._inFF = (A_SMA_FFControl(_X._inA10dD, _X._inB7dD, _X._inC7dD, _X._inYp_b1, _X._PstnDstExt, _X._inXp_b1))
									+ (((int32_t)CMD_REG->XKP*CMD_REG->ERRY)>>8)
									+ (((int32_t)CMD_REG->XKI*_X._inKIErr)>>8)
									+ (((int32_t)CMD_REG->XKD*(CMD_REG->ERRX- _X._inKDErr))>>8);

							_X._inKDErr = CMD_REG->ERRX;
							_X._inXp_b1 = _X._inFF;
							_X._inXp_b1 = _X._PstnDstExt;
							CMD_REG->XBFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERXB,_X._inFF, CMD_REG->YAPIEZOPWR);
							CMD_REG->XAFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERXA,-(_X._inFF), CMD_REG->YBPIEZOPWR);/*8.1us*/
				/*35.8us*/	CMD_REG->XAPIEZODUTY = A_SMA_Get_DutyXa(CMD_REG->XAFBPWR);
							CMD_REG->XBPIEZODUTY = A_SMA_Get_DutyXb(CMD_REG->XBFBPWR);
							A_SMA_Set_DutyX((CMD_REG->XAPIEZODUTY),0);
							A_SMA_Set_DutyX(0,(CMD_REG->XBPIEZODUTY));//yb_pwm
							setDutySeq = SMA_HW_YA;
							break;
					}
			}
	#endif
#elif(FEEDBACK_SELECTION == RESISTOR)
	if(adcRequestFlag == 1U)
	{
		adcRequestFlag = 0;
		switch(setDutySeq)
		{
		case SMA_HW_YA:
			PORT->GPIO0DT.BIT.PTD04 = 1;
			A_SMA_Set_DutyY(0x3fe,0);//A
			PORT->GPIO0DT.BIT.PTD04 = 0;
			setDutySeq = SMA_GET_ADC_YB;
			break;
		case SMA_GET_ADC_YB:
			#if (CHANNEL_SELECTION == CHANNEL_AF)
				CMD_REG->RYA_OUT = R_ADON_Get_ResisAF();
			#elif (CHANNEL_SELECTION == CHANNEL_Y)
				CMD_REG->RYA_OUT = R_ADON_Get_HallY();
			#elif (CHANNEL_SELECTION == CHANNEL_XY)
				CMD_REG->RYA_OUT = R_ADON_Get_HallY();
			#endif
			A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),(CMD_REG->YBPIEZODUTY));//y_pwm
			Vds_Y_A = ((CMD_REG->VDS_A * CMD_REG->RYA_OUT)>>(CMD_REG->EXT11-8)) + ((CMD_REG->VDS_B)>>(CMD_REG->EXT12-8));
			if(Vds_Y_A <= 0)
				Vds_Y_A = 0;
			setDutySeq = SMA_GET_ADC_XA;
			break;
		case SMA_GET_ADC_XA:
			A_SMA_Set_DutyY(0,0x3fe);//B
			setDutySeq = SMA_GET_DUTY_X;
			break;
		case SMA_GET_DUTY_X:
			#if (CHANNEL_SELECTION == CHANNEL_AF)
				CMD_REG->RYB_OUT = R_ADON_Get_ResisAF();
			#elif (CHANNEL_SELECTION == CHANNEL_Y)
				CMD_REG->RYB_OUT = R_ADON_Get_HallY();
			#elif (CHANNEL_SELECTION == CHANNEL_XY)
				CMD_REG->RYA_OUT = R_ADON_Get_HallY();
			#endif
			A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY),(CMD_REG->YBPIEZODUTY));//y_pwm
			Vds_Y_B = ((CMD_REG->VDS_A * CMD_REG->RYB_OUT)>>(CMD_REG->EXT11-8)) + ((CMD_REG->VDS_B)>>(CMD_REG->EXT12-8));
			if(Vds_Y_B <= 0)
				Vds_Y_B = 0;
			CMD_REG->FBY = A_SMA_Get_DrY();
			_Y._outPstnEst = A_SMA_MappingFunction_Cubic(CMD_REG->YMP_3, CMD_REG->YMP_2, CMD_REG->YMP_1,CMD_REG->FBY,CMD_REG->YMP_0, InvR, CMD_REG->EXT3);//4x
			setDutySeq = SMA_GET_ADC_YA;
			break;
		case SMA_GET_ADC_YA:
			/*-----------Estimation and setpoint from command extension--------------*/
			_Y._PstnDstExt = _Y._PstnDst<<Pwrfnpt;//Input_Ext(_Y._PstnDst , Pwrfnpt);//2^(ex4-ex3) for FF
			CMD_REG->ERRY = (int16_t)(_Y._PstnDst - _Y._outPstnEst);//for PID  2^(ex3)

			_Y._inKIErr += CMD_REG->ERRY;
			_Y._inFF = (A_SMA_FFControl(_Y._inA10dD, _Y._inB7dD, _Y._inC7dD, _Y._inYp_b1, _Y._PstnDstExt, _Y._inXp_b1))
					+ (((int32_t)CMD_REG->YKP*CMD_REG->ERRY)>>8)
					+ (((int32_t)CMD_REG->YKI*_Y._inKIErr)>>8)
					+ (((int32_t)CMD_REG->YKD*(CMD_REG->ERRY- _Y._inKDErr))>>8);

			_Y._inKDErr = CMD_REG->ERRY;

/*8.1us*/	 _Y._inYp_b1 = _Y._inFF;
			 _Y._inXp_b1 = _Y._PstnDstExt;
			CMD_REG->YBFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYB,_Y._inFF, CMD_REG->YAPIEZOPWR);
			CMD_REG->YAFBPWR = A_SMA_Power_Add(CMD_REG->MEANPOWERYA,-(_Y._inFF), CMD_REG->YBPIEZOPWR);/*8.1us*/
			setDutySeq = SMA_GET_ADC_XB;
			break;
		case SMA_GET_ADC_XB:
/*35.8us*/	CMD_REG->YAPIEZODUTY = A_SMA_Get_DutyYa(CMD_REG->YAFBPWR);
			CMD_REG->YBPIEZODUTY = A_SMA_Get_DutyYb(CMD_REG->YBFBPWR);
			setDutySeq = SMA_SET_DUTY_XB;
				break;
		case SMA_SET_DUTY_XB:
			if(rDutySetCount == 0)
			{
//				A_SMA_Set_DutyY((CMD_REG->YAPIEZODUTY<<1),(CMD_REG->YBPIEZODUTY<<1));//ya_pwm
				rDutySetCount++;
				setDutySeq = SMA_SET_DUTY_XB;
			}
			else if(rDutySetCount <= R_DUTYSET_COUNT)
			{
				rDutySetCount++;
				setDutySeq = SMA_SET_DUTY_XB;
			}
			else
			{
				rDutySetCount = 0;
				setDutySeq = SMA_HW_YA;
			}
			break;
		}
	}
#elif(PRODUCT_SELECTION == OIS)
#endif
}
#if(FEEDBACK_SELECTION == HALL)
/******************************************************************************
* Function Name: A_SMA_Get_DutyXA
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyXa(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
#if(FEEDBACK_SELECTION == RESISTOR)
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_X_P,CMD_REG->RXA_OUT), INIT_INVPXOPT, INIT_S);
#elif(FEEDBACK_SELECTION == HALL)
	DenominatorExt = INIT_POWER<<CMD_REG->EXT5;
#endif
	/*27.8us*/
	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOXTIME1.WORD);
	/*27.8us*/
	CMD_REG->XAPIEZOPWR = DenominatorExt;


return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyXB
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyXb(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
#if(FEEDBACK_SELECTION == RESISTOR)
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_X_N,CMD_REG->RXB_OUT), INIT_INVPXOPT, INIT_S);
#elif(FEEDBACK_SELECTION == HALL)
	DenominatorExt = INIT_POWER<<CMD_REG->EXT5;
#endif

	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOXTIME1.WORD);
	CMD_REG->XBPIEZOPWR = DenominatorExt;

return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyYA
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyYa(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
#if(FEEDBACK_SELECTION == RESISTOR)
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_Y_P,CMD_REG->RYA_OUT), INIT_INVPYOPT, INIT_S);
#elif(FEEDBACK_SELECTION == HALL)
	DenominatorExt = INIT_POWER<<CMD_REG->EXT5;
#endif


	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOYTIME1.WORD);
	CMD_REG->YAPIEZOPWR = DenominatorExt;

return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyYB
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyYb(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
#if(FEEDBACK_SELECTION == RESISTOR)
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_Y_N,CMD_REG->RYB_OUT), INIT_INVPYOPT, INIT_S);
#elif(FEEDBACK_SELECTION == HALL)
	DenominatorExt = INIT_POWER<<CMD_REG->EXT5;
#endif

	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOYTIME1.WORD);
	CMD_REG->YBPIEZOPWR = DenominatorExt;

return duty;
}
#elif(FEEDBACK_SELECTION == RESISTOR)
/******************************************************************************
* Function Name: A_SMA_Get_DutyXA
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyXa(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
	uint32_t vds;
	vds = (uint32_t)((((Vds_X_A*1000))/INIT_RS))<<CMD_REG->EXT10;
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs-vds, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_X_P,CMD_REG->RXA_OUT), INIT_INVPXOPT, INIT_S);
	/*27.8us*/
	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOXTIME1.WORD);
	/*27.8us*/
	CMD_REG->XAPIEZOPWR = DenominatorExt;


return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyXB
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyXb(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
	uint32_t vds;
	vds = (uint32_t)((((Vds_X_B*1000))/INIT_RS))<<CMD_REG->EXT10;
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs-vds, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_X_N,CMD_REG->RXB_OUT), INIT_INVPXOPT, INIT_S);
	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOXTIME1.WORD);
	CMD_REG->XBPIEZOPWR = DenominatorExt;

return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyYA
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyYa(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
	uint32_t vds;
	vds = (uint32_t)((((Vds_Y_A*1000))/INIT_RS))<<CMD_REG->EXT10;
	DenominatorExt = A_SMA_GetPrividedPower((PwrParam.ParamVmdRs-vds), INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_Y_P,CMD_REG->RYA_OUT), INIT_INVPYOPT, INIT_S);
//	DenominatorExt = 227<<CMD_REG->EXT5;
	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOYTIME1.WORD);
	CMD_REG->YAPIEZOPWR = DenominatorExt;

return duty;
}
/******************************************************************************
* Function Name: A_SMA_Get_DutyYB
* Description : This function get the VCM PWM duty.
* Arguments : ratio(0-100)
* Return Value : duty
******************************************************************************/
uint16_t A_SMA_Get_DutyYb(int32_t power)
{
	uint16_t duty;
	uint16_t DenominatorExt;
	uint32_t vds;
	vds = (uint32_t)((((Vds_X_B*1000))/INIT_RS))<<CMD_REG->EXT10;
	DenominatorExt = A_SMA_GetPrividedPower(PwrParam.ParamVmdRs-vds, INIT_1DRS ,A_SMA_GetVdiff(OIS_AXIS_Y_N,CMD_REG->RYB_OUT), INIT_INVPYOPT, INIT_S);
//	DenominatorExt = 227<<CMD_REG->EXT5;
	duty = A_SMA_DutyCal(power ,DenominatorExt , OISCNT->PIEZOYTIME1.WORD);
	CMD_REG->YBPIEZOPWR = DenominatorExt;

return duty;
}
#endif
/******************************************************************************
* Function Name: A_SMA_Target_Calculate
* Description : This function get the target of the unit of value.
* Arguments : target. the unit is command
* Return Value : get the target. the unit is physical
******************************************************************************/
void A_SMA_Target_Calculate(uint16_t targetX, uint16_t targetY)
{
		_X._PstnDst = A_SMA_Set_Target(CMD_REG->MINSTROKEX, CMD_REG->MAXSTROKEX, targetX);
		_Y._PstnDst = A_SMA_Set_Target(CMD_REG->MINSTROKEY, CMD_REG->MAXSTROKEY, targetY);
}
///******************************************************************************
//* Function Name: A_Resistor_Calibration_EXE
//* Description :ADC of Resistance Calibration processing
//* Arguments : none
//* Return Value : OK:calibration finish, NG:calibration during
//******************************************************************************/
//static uint8_t A_Resistor_Calibration_EXE(void)
//{
//	uint8_t ret = NG;
//	static uint16_t xAdcMin, xAdcMax, yAdcMin, yAdcMax;
//	if(AdcRequestFlag == 1U)   /* AdcRequestFlag is set by timerx interrupt(80us) */
//	{
//		switch(calibrationSeq)
//		{
//;
//		}
//	}
//	return( ret );
//}
#if(FEEDBACK_SELECTION == RESISTOR)
void A_SMA_Set_Duty_Disable(void)
{
	adcRequestFlag = 0U;
}
/******************************************************************************
* Function Name: R_HALL_Get2ndAmpGain
* Description : Get 2nd Amp gain value
* Arguments : axis
* Return Value : 2nd amp coefficient
******************************************************************************/
static uint32_t A_SMA_GetAmpGain()
{
    uint32_t AmpGainCoeff;
    int16_t tempHofs;

#if(CHANNEL_SELECTION == CHANNEL_X)
        tempHofs = (uint16_t)ANACNT->HAX_GAIN.BIT.HAX_GAIN;
#elif(CHANNEL_SELECTION == CHANNEL_Y)
        tempHofs = (uint16_t)ANACNT->HAY_GAIN.BIT.HAY_GAIN;
#elif(CHANNEL_SELECTION == CHANNEL_XY)
        tempHofs = (uint16_t)ANACNT->HAX_GAIN.BIT.HAX_GAIN;
        tempHofs = (uint16_t)ANACNT->HAY_GAIN.BIT.HAY_GAIN;		
#elif(CHANNEL_SELECTION == CHANNEL_AF)
        tempHofs = (uint16_t)ANACNT->HAAF_GAIN.BIT.HAAF_GAIN;
#endif
#if(CHANNEL_SELECTION == CHANNEL_X || CHANNEL_SELECTION == CHANNEL_Y|| CHANNEL_SELECTION == CHANNEL_XY)
    switch( tempHofs )
    {
        case (0x0000):
				AmpGainCoeff = 80;
        	break;
        case (0x0001):
				AmpGainCoeff = 120;
			break;
        case (0x0002):
				AmpGainCoeff = 160;
        	break;
        case (0x0004):
				AmpGainCoeff = 100;
        	break;
        case (0x0005):
				AmpGainCoeff = 150;
        	break;
        case (0x0008):
				AmpGainCoeff = 20;
            break;
        case (0x0009):
				AmpGainCoeff = 30;
            break;
        case (0x000A):
				AmpGainCoeff = 40;
            break;
        case (0x000B):
				AmpGainCoeff = 50;
            break;
    }
#elif(CHANNEL_SELECTION == CHANNEL_AF)
    switch( tempHofs )
        {
            case (0x0001):
    				AmpGainCoeff = 7;
    			break;
            case (0x0002):
    				AmpGainCoeff = 12;
            	break;
            case (0x0004):
    				AmpGainCoeff = 27;
            	break;
            case (0x000E):
    				AmpGainCoeff = 95;
                break;
            case (0x0012):
    				AmpGainCoeff = 41;
                break;
            case (0x0015):
    				AmpGainCoeff = 126;
                break;
            case (0x0019):
    				AmpGainCoeff = 47;
                break;
            case (0x001A):
    				AmpGainCoeff = 80;
                break;
        }
#endif
    return( AmpGainCoeff );
}
/******************************************************************************
* Function Name: A_SMA_Get2ndAmpGain
* Description : Get 2nd Amp gain value
* Arguments : axis
* Return Value : 2nd amp coefficient
******************************************************************************/
static uint32_t A_SMA_Get2ndAmpGain()
{
	uint32_t SecondAmpGainCoeff;
    int16_t tempHofs;

#if(CHANNEL_SELECTION == CHANNEL_X)
        tempHofs = (uint16_t)ANACNT->HAX_GAIN.BIT.HAX_GAIN;
#elif(CHANNEL_SELECTION == CHANNEL_Y)
        tempHofs = (uint16_t)ANACNT->HAY_GAIN.BIT.HAY_GAIN;
#elif(CHANNEL_SELECTION == CHANNEL_XY)
        tempHofs = (uint16_t)ANACNT->HAX_GAIN.BIT.HAX_GAIN;
        tempHofs = (uint16_t)ANACNT->HAY_GAIN.BIT.HAY_GAIN;				
#elif(CHANNEL_SELECTION == CHANNEL_AF)
        tempHofs = (uint16_t)ANACNT->HAAF_GAIN.BIT.HAAF_GAIN;
#endif
#if(CHANNEL_SELECTION == CHANNEL_X || CHANNEL_SELECTION == CHANNEL_Y|| CHANNEL_SELECTION == CHANNEL_XY)
    switch( tempHofs )
    {
        case (0x0000):
        case (0x0004):
        case (0x0008):
            SecondAmpGainCoeff = 571; /* 2nd HallAmp gain x5.71 */
            break;

        case (0x0001):
        case (0x0005):
        case (0x0009):
            SecondAmpGainCoeff = 857; /* 2nd HallAmp gain x8.57 */
            break;

        case (0x0002):
        case (0x0006):
        case (0x000A):
            SecondAmpGainCoeff = 1143; /* 2nd HallAmp gain x11.43 */
            break;

        case (0x0003):
        case (0x0007):
        case (0x000B):
        default:
            SecondAmpGainCoeff = 1429; /* 2nd HallAmp gain x14.29 */
            break;
    }
#elif(CHANNEL_SELECTION == CHANNEL_AF)
    switch( tempHofs )
    {
        case (0x0001):
        case (0x0002):
        case (0x0004):
            SecondAmpGainCoeff = 300; /* 2nd HallAmp gain x3 */
            break;

        case (0x000E):
            SecondAmpGainCoeff = 500; /* 2nd HallAmp gain x5 */
            break;

        case (0x0012):
        case (0x0015):
            SecondAmpGainCoeff = 1020; /* 2nd HallAmp gain x10.20 */
            break;

        case (0x0019):
        case (0x001A):
        default:
            SecondAmpGainCoeff = 2000; /* 2nd HallAmp gain x20 */
            break;
    }
#endif
    return( SecondAmpGainCoeff );
}

/******************************************************************************
* Function Name: A_PWR_CalcParamChange
* Description : This function performs a calculation at the time of power parameters change
* Arguments :ois_axis_select_e axis
* 			 using SMA wire to make a product the axis should be set to
*            gyro_targetAll_t target
* Return Value : none
******************************************************************************/
void A_PWR_CalcParamChangeX(void)
{
#if(CHANNEL_SELECTION == CHANNEL_X || CHANNEL_SELECTION == CHANNEL_Y || CHANNEL_SELECTION == CHANNEL_XY)
	PwrParam.ParamRsG1G2s12_X = (uint32_t)((INIT_RS*A_SMA_GetAmpGain()/10)<<(12+CMD_REG->EXT2));//C
	PwrParam.ParamG2OfstG2Vm_X = (uint32_t)((((A_SMA_Get2ndAmpGain())<<11)/100)+(1<<11)-((CMD_REG->XHOFS*A_SMA_Get2ndAmpGain())<<4)/100);
	PwrParam.ParamInvP_X = (uint32_t)((((A_SMA_GetAmpGain())<<12)/INIT_VM)*10);
	PwrParam.ParamVmdRs = (uint32_t)((((INIT_VM*1000))/INIT_RS))<<CMD_REG->EXT10;
#elif(CHANNEL_SELECTION == CHANNEL_AF)
	PwrParam.ParamRsG1G2s12_X = (uint32_t)((INIT_RS*A_SMA_GetAmpGain()/10)<<(12+CMD_REG->EXT2));//C
	PwrParam.ParamG2OfstG2Vm_X = (1<<11)+(((((A_SMA_Get2ndAmpGain()/100)<<12)*10/INIT_VM)*((((CMD_REG->XHOFS-128))*INIT_VM)/10)>>8));
	PwrParam.ParamInvP_X = (uint32_t)((((A_SMA_GetAmpGain())<<12)/INIT_VM)*10);
	PwrParam.ParamVmdRs = (uint32_t)((((INIT_VM*1000))/INIT_RS))<<CMD_REG->EXT10;
#endif
}

/******************************************************************************
* Function Name: A_PWR_CalcParamChange
* Description : This function performs a calculation at the time of power parameters change
* Arguments :ois_axis_select_e axis
*            gyro_targetAll_t target
* Return Value : none
******************************************************************************/
void A_PWR_CalcParamChangeY(void)
{
#if(CHANNEL_SELECTION == CHANNEL_X || CHANNEL_SELECTION == CHANNEL_Y || CHANNEL_SELECTION == CHANNEL_XY)
	PwrParam.ParamRsG1G2s12_Y = (uint32_t)((INIT_RS*A_SMA_GetAmpGain()/10)<<(12+CMD_REG->EXT2));
	PwrParam.ParamG2OfstG2Vm_Y = (uint32_t)((((A_SMA_Get2ndAmpGain())<<11)/100)+(1<<11)-((CMD_REG->YHOFS*A_SMA_Get2ndAmpGain())<<4)/100);
	PwrParam.ParamInvP_Y = (uint32_t)((((A_SMA_GetAmpGain())<<12)/INIT_VM)*10);
	PwrParam.ParamVmdRs = (uint32_t)((((INIT_VM*1000))/INIT_RS))<<CMD_REG->EXT10;
#elif(CHANNEL_SELECTION == CHANNEL_AF)
	PwrParam.ParamRsG1G2s12_Y = (uint32_t)((INIT_RS*A_SMA_GetAmpGain()/10)<<(12+CMD_REG->EXT2));
	PwrParam.ParamG2OfstG2Vm_Y = (1<<11)+(((((A_SMA_Get2ndAmpGain()/100)<<12)*10/INIT_VM)*((((CMD_REG->XHOFS-128))*INIT_VM)/10)>>8));
	PwrParam.ParamInvP_Y = (uint32_t)((((A_SMA_GetAmpGain())<<12)/INIT_VM)*10);
	PwrParam.ParamVmdRs = (uint32_t)((((INIT_VM*1000))/INIT_RS))<<CMD_REG->EXT10;
#endif
}
#endif

