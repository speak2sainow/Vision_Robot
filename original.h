#ifndef _ORIGINAL_H
#define _ORIGINAL_H

#include "Arduino.h"

#define REG_TC0_BMR   *(volatile unsigned int*) 0x400800C4 /**< \brief (TC0) Block Mode Register */
#define REG_TC0_CMR0  *(volatile unsigned int*) 0x40080004 /**< \brief (TC0) Channel Mode Register (channel = 0) */
#define REG_TC0_CCR0  *(volatile unsigned int*) 0x40080000 /**< \brief (TC0) Channel Control Register (channel = 0) */
#define REG_TC0_CV0   *(volatile unsigned int*) 0x40080010 /**< \brief (TC0) Counter Value (channel = 0) */
#define REG_TC2_BMR   *(volatile unsigned int*) 0x400880C4 /**< \brief (TC2) Block Mode Register */
#define REG_TC2_CMR0  *(volatile unsigned int*) 0x40088004 /**< \brief (TC2) Channel Mode Register (channel = 0) */
#define REG_TC2_CCR0  *(volatile unsigned int*) 0x40088000 /**< \brief (TC2) Channel Control Register (channel = 0) */
#define REG_TC2_CV0   *(volatile unsigned int*) 0x40088010 /**< \brief (TC2) Counter Value (channel = 0) */
#define REG_PMC_PCER0 *(volatile unsigned int*) 0x400E0610 /**< \brief PMC Peripheral Clock Enable Register 0 */
#define REG_PMC_PCER1 *(volatile unsigned int*) 0x400E0700 /**< \brief PMC Peripheral Clock Enable Register 1 */
#define REG_ADC_CGR   *(volatile unsigned int*) 0x400C0048
#define REG_ADC_MR   *(volatile unsigned int*) 0x400C0004  //ADC Mode Register
#define REG_ADC_CHER   *(volatile unsigned int*) 0x400C0010 //ADC Channel Enable Register
#define REG_ADC_CR   *(volatile unsigned int*) 0x400C0000
#define REG_ADC_ISR   *(volatile unsigned int*) 0x400C0030
#define REG_ADC_LCDR   *(volatile unsigned int*) 0x400C0020 //ADC Last Converted Data Register
#define ADC_MOTOR_1		*(volatile unsigned int*)(0x400c0050)
#define ADC_MOTOR_2		*(volatile unsigned int*)(0x400c0054)

#define PWM_MOTOR1 1<<24
#define PWM_MOTOR2 1<<23
#define PWM_CONTROLLER_MOTOR_1 p_Pwm->PWM_CH_NUM[7].PWM_CDTY
#define PWM_CONTROLLER_MOTOR_2 p_Pwm->PWM_CH_NUM[6].PWM_CDTY
#define ENCODER_MOTOR_1 REG_TC0_CV0
#define ENCODER_MOTOR_2 REG_TC2_CV0
#define CS_MOTOR_1_INB 0x1
#define CS_MOTOR_1_INA 0x2
#define CS_MOTOR_2_INB 0x4
#define CS_MOTOR_2_INA 0x8
//EVENT LINE THINGS
//PWM Interrupt Enable Register 2
#define REG_PWM_IER2 *(volatile unsigned int*) 0x40094034
//PWM Comparison x Mode Register
#define REG_PWM_CMPM7 *(volatile unsigned int*) 0x400941A8
#define REG_PWM_CMPM6 *(volatile unsigned int*) 0x40094198
//PWM Comparison x Value Register
#define REG_PWM_CMPV7 *(volatile unsigned int*) 0x400941A0
#define REG_PWM_CMPV6 *(volatile unsigned int*) 0x40094190
//PWM Event Line 0 Register
#define REG_PWM_ELMR0 *(volatile unsigned int*) 0x4009407C
//PWM Enable Register
#define REG_PWM_ENA *(volatile unsigned int*) 0x40094004

//Timer counter for sensor
//TC1 Channel 0 Mode Register: Capture Mode
#define REG_TC1_CMR0 *(volatile unsigned int*) 0x40084004
//TC1 Channel 0 Control Register
#define REG_TC1_CCR0 *(volatile unsigned int*) 0x40084000
//TC Counter Value Register
#define REG_TC1_CV0 *(volatile unsigned int*) 0x40084010

//ADC Interrupt Enable Register
#define REG_ADC_IER *(volatile unsigned int*) 0x400C0024
//ADC Interrupt Disable Register
#define REG_ADC_IDR *(volatile unsigned int*) 0x400C0028
//PWM ISR2
#define REG_PWM_ISR2 *(volatile unsigned int*) 0x40094040

/**********************GLOBAL FUNCTIONS*************************/
void Init_Sense(void);
void Init_PIOC_Interrupt(void);
void Pulse(void);
int printdist(void);
void EVENT(void);
void EVENT_off(void);
void ADC_IE(void);
void ADC_ID(void);
//Peripheral identifier
#define ADC_PID 37
void init_NVIC_ADC(void);
void incPos(void);
/*************************UDOO CLASS****************************/

class UDOO
{
public:
	UDOO();
	void Init(void);
	void Set_dist(unsigned int);
	void Set_back(unsigned int);
	void Position_Control_Loop(void);
	void Speed_Control_Loop(void);
	void Current_Control_Loop(void);
	int Desired_Position_Motor1;
	int Desired_Position_Motor2;
	void Pos_Planner(void);
	void back_Planner(void);
   //plot
   void plotting();
   double dist_sense[158];
   double enc_val[158];
   int count=0;//Distance sensor and encoder count
   double getplot(int,int);
   int plotdone=0;
   //plot
   //rotation
   void rotate_Right(void);
   void rotate_Left(void);
   void degr_left(int);
   void degr_right(int);
   int rotdist_left=0;
   int rotdist_right=0;
   int ritdone = 0;
   int leftdone = 0;
   int posdone = 0;
   int backdone=0;
   //rotation
	double Current_Motor1;
	double Current_Motor2;
	double Desired_Current_Motor1;
	double Desired_Current_Motor2;
	int Control_Current_Motor_1;
	int Control_Current_Motor_2;
   double Desired_Speed_Motor1;

private:

	unsigned long Next_Current_Control_Loop;
	unsigned long Next_Speed_Control_Loop;
	unsigned long Next_Position_Control_Loop;
	unsigned long Micros_Now;
	unsigned long Millis_Now;
	unsigned long Current_Loop_Interval; // default 1000us = 1ms
	unsigned long Speed_Loop_Interval; // default 10ms
	unsigned long Position_Loop_Interval; // default 100ms
	int Old_Pos_Motor_1;
	int Old_Pos_Motor_2;
	int Actual_Pos_Motor1;
	int Actual_Pos_Motor2;
	double Actual_Speed_Motor1;
	double Actual_Speed_Motor2;
	//double Desired_Speed_Motor1;
	double Desired_Speed_Motor2;
	double Diff_Speed_Motor1;
	double Diff_Speed_Motor2;
	double Sum_Speed_Motor1;
	double Sum_Speed_Motor2;
	/************************POSITION PLANNER***************************/
	double temp_Desired_Position_Motor1=0;
	double temp_Desired_Position_Motor2=0;
	double MaxSpeed = 216;//75% of Max permissable speed : 119.333333333*0.75=89.5rps-> multiplied with circumference of wheel(94.2mm)/39(ratio)->216mm/s.
	double MaxAcc = 50; //Maximum acceleration
	double PlanSpeed=0;//Planned speed (Used for position incrementation)
	double BreakPos=0;
	unsigned long PosError =0;
	double MmPos1=0;
	double MmPos2=0;
};
/************************Clone FUNCTIONS***************************/
void copy(UDOO);
UDOO copy2(void);
UDOO *copypoint();
#endif //_ORIGINAL_H
