#include "original.h"
#include "Arduino.h"
#include "math.h"


//Copy Arduino IDE UDOO object to clone variable
UDOO clone;
UDOO *point=&clone;


/******************AVSTANDSSENSOR VARIABLER*********************/
//Check ISR
int clear = 0;
//Timer Counter value
int CV = 0;
//Switcheroo
int switcheroo = -1;
//distance value
double dist = 0.0;
double trk=0;
//for going back
double go_back = 0.0;
double brk=0;
//closed loop ticks
int curr_tick,pos_tick,speed_tick,plan_tick,plot_tick,rot_tick = 0;
int dir=0;
int sonn;



/************************GLOBAL Functions***************************/



//Initera sensor
void Init_Sense(void){

	volatile Pmc * p_PMC = (Pmc *)PMC;
	p_PMC->PMC_PCER0 = 1<<13 | 1<<30;
	//PIOC for sensor *
	volatile Pio * p_Pio_C = (Pio *)PIOC;
	//PIOC Enable Register PC19 TRIGGER(digital pin44 to pulse the sensor) *
	p_Pio_C->PIO_PER = 1<<19;
	//PIOC Disable Register PC18 ECHO(digital pin45 read from sensor) *
	p_Pio_C->PIO_ODR = 1<<18;
	//Set the timer clock for TC1 (TIMER_CLOCK2/TCLK2) for sensor *
	REG_TC1_CMR0 = 0x1;//1:MCK/8

	//For sensor *
	//Setup PIOC interrupt *
	//Disable the selected Interrupt *
	NVIC_DisableIRQ((IRQn_Type)13);
	//Clearing any pending interrupt
	NVIC_ClearPendingIRQ((IRQn_Type)13);
	//Set priority of the interrupt
	NVIC_SetPriority((IRQn_Type)13,0);
	//Enable irq interrupt
	NVIC_EnableIRQ((IRQn_Type)13);
}

//Init interrupt for PIOC
void Init_PIOC_Interrupt(void){

	//PIOC for sensor *
	volatile Pio * p_Pio_C = (Pio *)PIOC;
	//Disable PIOC interrupts
	p_Pio_C->PIO_IDR = 0xffffffff;
	//Enable PIOC interrupt
	p_Pio_C->PIO_IER = 1<<18;
	//Additional Interrupt Modes Enable Register (PIOC)
	p_Pio_C->PIO_AIMER = 1<<18;
	//The interrupt source is an Edge detection event
	p_Pio_C->PIO_ESR = 1<<18;
	//The interrupt source is set to a Rising Edge detection
	p_Pio_C->PIO_REHLSR = 1<<18;
}

void PIOC_Handler(){

	volatile Pio * p_Pio_C = (Pio *)PIOC;
	//Clear interrupts
	clear = p_Pio_C->PIO_ISR;
	clear &= p_Pio_C->PIO_IMR;

	if(switcheroo == -1)
	{
	  //Reset and start clock
	  REG_TC1_CCR0 = 0x5;
	  //Change to falling edge interrupt
	  p_Pio_C->PIO_FELLSR = 1<<18;
	  //Switcheroo
	  switcheroo *= -1;
	}
	else if(switcheroo == 1)
	{
	  //Save counter value
	  CV = REG_TC1_CV0;
	  //Calculate distance from time - 84 000 000 / 8 = 1 tick
	  sonn = (((1.0/(84000000.0/8))*CV)*34000)/2.0;//mm (((1.0/(84000000.0/8))*CV)*34000)/2.0; //cm //original measurement from ultrasound
	  //Change to rising edge interrupt
	  p_Pio_C->PIO_REHLSR = 1<<18;
	  //Disable the clock
	  REG_TC1_CCR0 = 0x1;
	  //Switcheroo
	  switcheroo *= -1;
	}
}

//Generate TTL pulse (trigger) on PC19
void Pulse(void){

	//PIOC for sensor *
	volatile Pio * p_Pio_C = (Pio *)PIOC;
	p_Pio_C->PIO_OER = 1<<19;
	//Low for approximately 10 us
	delayMicroseconds(20);
	p_Pio_C->PIO_ODR = 1<<19;
}

//Print distance of sensor
int printdist(){

	return sonn;
}

/*********CLONE reach UDOO variables********/
//Copy UDOO object from Arduino IDE to clone
void copy(UDOO insert){

	clone = insert;
}

//Return UDOO clone to Arduino IDE
UDOO copy2(){

	return clone;
}
UDOO *copypoint(){
	return &clone;
}



/****************TRIGGER EVENT LINE******************/
void EVENT(void){

	volatile Pwm * p_Pwm = (Pwm *)PWM;
	//Enable Hardware interrupt and select PWM event line 0
	REG_ADC_MR |= 0x9;

	//Config PWM channel 0
	p_Pwm->PWM_CH_NUM[0].PWM_CMR = 0x0; //Clear Pre-scaler (MCK)
	p_Pwm->PWM_CH_NUM[0].PWM_CPRD = 4200; //Set Channel Period (MCK/20k)
	p_Pwm->PWM_CH_NUM[0].PWM_CDTY = 0; //Clear Channel Duty-Cycle

	//Enable channel 6 and 7 compare interrupt
	REG_PWM_IER2 = 0x8000;
	//Enable compare mode on channel 7
	REG_PWM_CMPM7 = 0x1;
	//Enable compare mode on channel 6
	//REG_PWM_CMPM6 = 0x1;
	//Set channel 7 value to compare (compares with channel 0's counter value)
	REG_PWM_CMPV7 = 0x4;
	//Set channel 6 value to compare (compares with channel 0's counter value)
	//REG_PWM_CMPV6 = 0x4;
	//Select comparsion y(csel7 and csel6) on event line 0
	REG_PWM_ELMR0 = 0x80;

	p_Pwm->PWM_ENA = PWM_ENA_CHID0; //Enable PWM output for channel 0
}

void EVENT_off(void){

	volatile Pwm * p_Pwm = (Pwm *)PWM;
	REG_PWM_ELMR0 = 0x0;
	p_Pwm->PWM_CH_NUM[6].PWM_CDTY = 0;
	p_Pwm->PWM_CH_NUM[7].PWM_CDTY = 0;
}
//ADC Enable interrupt
void ADC_IE(void){

  //Disable end of conversion interrupt on channels
  REG_ADC_IDR = 0xf00ffff;
  //Enable end of conversion interrupt on channel 0
  REG_ADC_IER = 0x1;
}
//ADC Disable interrupt
void ADC_ID(void){

  //Disable end of conversion interrupt on channel 0
  REG_ADC_IDR = 0x1;
}

//Init NVIC for ADC
void init_NVIC_ADC(void){
  //NVIC setup
  //Disable the selected Interrupt
  NVIC_DisableIRQ((IRQn_Type)ADC_PID);
  //Clearing any pending interrupt
  NVIC_ClearPendingIRQ((IRQn_Type)ADC_PID);
  //Set priority of the interrupt
  NVIC_SetPriority((IRQn_Type)ADC_PID,0);
  //Enable irq interrupt
  NVIC_EnableIRQ((IRQn_Type)ADC_PID);
}

//Current control loop
void ADC_Handler(void){

	pos_tick++;
	speed_tick++;
	curr_tick++;
	plan_tick++;

	rot_tick++;
	plot_tick++;
	if(curr_tick>=4){

     		curr_tick = 0;

		volatile Pwm * p_Pwm = (Pwm *)PWM;
		volatile Pio * p_Pio_D = (Pio *)PIOD;

			// AD_Value * 3.3 /4095 -> [volt]. PGA = 4 and I_out/I_sense = 7000 and R_sense = 1kOhm
			// AD_Value * 3.3 /4095/4/1000*7000 == AD_Value*0.00141
		clone.Current_Motor1 = ADC_MOTOR_1* 0.00141; // convert to Ampere
		clone.Current_Motor2 = ADC_MOTOR_2* 0.00141;

		if(clone.Desired_Current_Motor1 > 0)
			clone.Control_Current_Motor_1 = (int)399*(clone.Desired_Current_Motor1 - clone.Current_Motor1); // P-Controller
		else
			clone.Control_Current_Motor_1 = (int)399*(clone.Desired_Current_Motor1 + clone.Current_Motor1); // P-Controller
		if(clone.Desired_Current_Motor2 > 0)
			clone.Control_Current_Motor_2 = (int)399*(clone.Desired_Current_Motor2 - clone.Current_Motor2); // P-Controller
		else
			clone.Control_Current_Motor_2 = (int)399*(clone.Desired_Current_Motor2 + clone.Current_Motor2); // P-Controller

		if(clone.Control_Current_Motor_1 < 0){
			if(clone.Control_Current_Motor_1 < -4200) // 4200 = Max PWM Dutycycle
				clone.Control_Current_Motor_1 = -4200;
			p_Pio_D->PIO_CODR = CS_MOTOR_1_INB; // Set dir
			p_Pio_D->PIO_SODR = CS_MOTOR_1_INA;
			PWM_CONTROLLER_MOTOR_1 = -clone.Control_Current_Motor_1; // Set PWM
		}
		else{
			if(clone.Control_Current_Motor_1 > 4200) // 4200 = Max PWM Dutycycle
				clone.Control_Current_Motor_1 = 4200;
			p_Pio_D->PIO_CODR = CS_MOTOR_1_INA;// Set dir
			p_Pio_D->PIO_SODR = CS_MOTOR_1_INB;
			PWM_CONTROLLER_MOTOR_1 = clone.Control_Current_Motor_1; // Set PWM
		}
		if(clone.Control_Current_Motor_2 < 0){
			if(clone.Control_Current_Motor_2 < -4200) // 4200 = Max PWM Dutycycle
				clone.Control_Current_Motor_2 = -4200;
			p_Pio_D->PIO_CODR = CS_MOTOR_2_INB;// Set dir
			p_Pio_D->PIO_SODR = CS_MOTOR_2_INA;
			PWM_CONTROLLER_MOTOR_2 = -clone.Control_Current_Motor_2; // Set PWM
		}
		else{
			if(clone.Control_Current_Motor_2 > 4200) // 4200 = Max PWM Dutycycle
				clone.Control_Current_Motor_2 = 4200;
			p_Pio_D->PIO_CODR = CS_MOTOR_2_INA;// Set dir
			p_Pio_D->PIO_SODR = CS_MOTOR_2_INB;
			PWM_CONTROLLER_MOTOR_2 = clone.Control_Current_Motor_2; // Set PWM
		}
	}
}
UDOO::UDOO(void){
}


void UDOO::Init(void){

	volatile Pmc * p_PMC = (Pmc *)PMC;
    volatile Pwm * p_Pwm = (Pwm *)PWM;
	volatile Pio * p_Pio_D = (Pio *)PIOD;


	REG_PMC_PCER0 = 1<<11 | 1<< 12 | 1<<13 | 1<< 14 | 1 << 27 | 1 << 28 | 1 << 29;
	REG_PMC_PCER1 = 0x0f;

	p_PMC->PMC_PCER1 = 0x01 << (ID_PWM-32);    // enable peripheral clk pid = 36 = PWM
    p_PMC->PMC_PCER1 = 0x01 << (ID_ADC-32);    // enable peripheral clk pid = 37 = ADC
	p_Pio_D->PIO_PER = 0xf; //enable PIOD port 0-3
	p_Pio_D->PIO_CODR = 0xf;//clear output port 0-3
	p_Pio_D->PIO_OER = 0xf; //output enable PIOD port 0-3
	p_Pio_D->PIO_PUDR = 0xf; //disable pullup PIOD port 0-3

   //PWM motor 1
   REG_PIOC_PDR = PWM_MOTOR1; // Let peripheral control port C pin 24
   REG_PIOC_ABSR = REG_PIOC_ABSR | PWM_MOTOR1; // Activate peripheral B to control the pin
   p_Pwm->PWM_ENA = PWM_ENA_CHID7;
   p_Pwm->PWM_CH_NUM[7].PWM_CMR = 0x0;
   p_Pwm->PWM_CH_NUM[7].PWM_CPRD = 4200;
   p_Pwm->PWM_CH_NUM[7].PWM_CDTY = 00;
   //PWM motor 2
   REG_PIOC_PDR = PWM_MOTOR2; // Let peripheral control port C pin 24
   REG_PIOC_ABSR = REG_PIOC_ABSR | PWM_MOTOR2; // Activate peripheral B to control the pin
   p_Pwm->PWM_ENA = PWM_ENA_CHID6;
   p_Pwm->PWM_CH_NUM[6].PWM_CMR = 0x0;
   p_Pwm->PWM_CH_NUM[6].PWM_CPRD = 4200;
   p_Pwm->PWM_CH_NUM[6].PWM_CDTY = 000;
	REG_ADC_MR = 0x03000200; // prescaler = 2 => 84MHz/6 = 14MHz ADC_Clk
	REG_ADC_CGR = 0x00000003; //PGA = 4
	REG_ADC_CHER = 0x3; //enable ch0 and ch1

	REG_TC0_BMR = 3 << 8; // enable counting & quadrature  PB25 , PB27
	REG_TC0_CMR0 = 0x05; //select XC0
	REG_TC0_CCR0 = 0x05; // SWTRG & CLKEN
	REG_TC2_BMR = 3 << 8; // enable counting & quadrature  PC25 , PC26
	REG_TC2_CMR0 = 0x05; //select XC0
	REG_TC2_CCR0 = 0x05; // SWTRG & CLKEN

	Next_Current_Control_Loop = 0;
	Next_Speed_Control_Loop = 0;
	Next_Position_Control_Loop = 0;
	Current_Loop_Interval = 250; // default 250us
	Speed_Loop_Interval = 1000; // default 1000us
	Position_Loop_Interval = 50; // default 50ms
	Desired_Current_Motor1 = 0.0;
	Desired_Current_Motor2 = 0.0;

	Old_Pos_Motor_1 = ENCODER_MOTOR_1;
	Old_Pos_Motor_2 = ENCODER_MOTOR_2;
	Desired_Speed_Motor1 = 0;
	Desired_Speed_Motor2 = 0;
	Desired_Position_Motor1 = 0;
	Desired_Position_Motor2 = 0;
	Sum_Speed_Motor1 = 0.0;
	Sum_Speed_Motor2 = 0.0;
}

void UDOO::Speed_Control_Loop(void) {
  if (speed_tick>=80) {
	speed_tick=0;
    Next_Speed_Control_Loop = Next_Speed_Control_Loop + Speed_Loop_Interval;

	Actual_Pos_Motor1 = ENCODER_MOTOR_1;
	Actual_Pos_Motor2 = ENCODER_MOTOR_2;
	Actual_Speed_Motor1 = (double)(Actual_Pos_Motor1 - Old_Pos_Motor_1);
	Actual_Speed_Motor2 = (double)(Actual_Pos_Motor2 - Old_Pos_Motor_2);
	Old_Pos_Motor_1 = Actual_Pos_Motor1;
	Old_Pos_Motor_2 = Actual_Pos_Motor2;
	Diff_Speed_Motor1 = Desired_Speed_Motor1 - Actual_Speed_Motor1;
	Diff_Speed_Motor2 = Desired_Speed_Motor2 - Actual_Speed_Motor2;

	Sum_Speed_Motor1 += Diff_Speed_Motor1; // Integrate
	if(Sum_Speed_Motor1 > 300) // Antiwindup
		Sum_Speed_Motor1 = 300;
	else if(Sum_Speed_Motor1 < -300)
		Sum_Speed_Motor1 = -300;

	Sum_Speed_Motor2 += Diff_Speed_Motor2; // Integrate
	if(Sum_Speed_Motor2 > 300) // Antiwindup
		Sum_Speed_Motor2 = 300;
	else if(Sum_Speed_Motor2 < -300)
		Sum_Speed_Motor2 = -300;

	 Desired_Current_Motor1 = 0.1*(Diff_Speed_Motor1) + 0.01*Sum_Speed_Motor1; // PI-Controller
	 Desired_Current_Motor2 = 0.1*(Diff_Speed_Motor2) + 0.01*Sum_Speed_Motor2; // PI-Controller

	if(Desired_Current_Motor1<-3.0) // Saturation
		Desired_Current_Motor1 = -3.0;
	else if(Desired_Current_Motor1 > 3.0)
		Desired_Current_Motor1 = 3.0;

	if(Desired_Current_Motor2<-3.0) // Saturation
		Desired_Current_Motor2 = -3.0;
	else if(Desired_Current_Motor2 > 3.0)
		Desired_Current_Motor2 = 3.0;
  }
}

void UDOO::Set_dist(unsigned int len){
	     
	    trk=len;
	    Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;
	
	dist=trk+(MmPos1+MmPos2)/2;
	}

void UDOO::Pos_Planner(){
	if(plan_tick>=1500){
		plan_tick=0;
		Pulse();
		Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;

temp_Desired_Position_Motor1 = MmPos1;
temp_Desired_Position_Motor2 = MmPos2;

        PosError=dist-(MmPos1+MmPos2)/2 ; //desired position- mean of traveled distance by wheels
		if(abs(PosError)<10){
		PosError=0;
		}

BreakPos=(PlanSpeed*PlanSpeed)/(2*MaxAcc);


	if(PosError-BreakPos>0){
		if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);//Increase PlanSpeed with maximum acceleration multiplied by sampling time for Pos_Planner
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}

	else if(PosError+BreakPos<0){
		if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed>5){
		if(PlanSpeed>0){
			if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed<0){
			if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}
	

	}
	else{
		posdone=1;
		}

	 	Desired_Position_Motor1=(int)(temp_Desired_Position_Motor1*211.974522293); //Pulses per wheel round:512*39=19968 -> ||Desired_Position_Motor1=(int)(0.9*(EndPos*211.974522293)+(0.05*testsum*211.974522293));//
        	Desired_Position_Motor2=(int)(temp_Desired_Position_Motor2*211.974522293);

	}
}
	
void UDOO::Set_back(unsigned int ben){
	     
	    brk=ben;
	    Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;
	
	go_back=(MmPos1+MmPos2)/2-brk;
	}

void UDOO::back_Planner(){
	if(plan_tick>=1500){
		plan_tick=0;
		Pulse();
		Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;

temp_Desired_Position_Motor1 = MmPos1;
temp_Desired_Position_Motor2 = MmPos2;

        PosError=(MmPos1+MmPos2)/2 -go_back; //desired position- mean of traveled distance by wheels
		if(abs(PosError)<10){
		PosError=0;
		}

BreakPos=(PlanSpeed*PlanSpeed)/(2*MaxAcc);


	if(PosError-BreakPos>0){
		if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);//Increase PlanSpeed with maximum acceleration multiplied by sampling time for Pos_Planner
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}

	else if(PosError+BreakPos<0){
		if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed>5){
		if(PlanSpeed>0){
			if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed<0){
			if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}
	

	}
	else{
		backdone=1;
		}

	 	Desired_Position_Motor1=(int)(temp_Desired_Position_Motor1*211.974522293); //Pulses per wheel round:512*39=19968 -> ||Desired_Position_Motor1=(int)(0.9*(EndPos*211.974522293)+(0.05*testsum*211.974522293));//
        	Desired_Position_Motor2=(int)(temp_Desired_Position_Motor2*211.974522293);

	}


}//-> wheel circumference:30*3.14=94.2mm-> 1mm:19968/94.2=211.974522293 pulses||Desired_Position_Motor2=(int)(EndPos*211.974522293);//



void UDOO::plotting(){
	if(plot_tick>=5000 && plotdone==0){
	plot_tick=0;

        if(count<158){
		dist_sense[count]=dist;
		enc_val[count]=Actual_Pos_Motor1;
		enc_val[count]*=0.004717548;
		count++;
		}else if(count==158){
		    plotdone=1;
		}


	}
}
double UDOO::getplot(int choice,int i){
    if(choice==1){
    return dist_sense[i];
    }else if(choice==2){
    return enc_val[i];
    }else{
    return 0.0;
    }
}
void UDOO::degr_left(int degl){   //Before it is INT
	    
	    rotdist_left=0;
	    //Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		//Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;
	
    rotdist_left+=(degl*1.918888889)+MmPos1;
    
   
}

void UDOO::degr_right(int degr){ 	//Before it is INT
	    
	    rotdist_right=0;
	    //Actual_Pos_Motor1 = ENCODER_MOTOR_1;
		//Actual_Pos_Motor2 = ENCODER_MOTOR_2;
		
		MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
		MmPos2=Actual_Pos_Motor2*0.004717548;
	
    rotdist_right+=(degr*1.918888889)+MmPos2;
    
   
}

void UDOO::rotate_Left(){
	if(rot_tick>=1500){
		rot_tick=0;

    MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
    MmPos2=Actual_Pos_Motor2*0.004717548;

temp_Desired_Position_Motor1 = MmPos1;
temp_Desired_Position_Motor2 = MmPos2;

BreakPos=(PlanSpeed*PlanSpeed)/(2*MaxAcc);


	if((rotdist_left-temp_Desired_Position_Motor1-BreakPos)>0){
		if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);//Increase PlanSpeed with maximum acceleration multiplied by sampling time for Pos_Planner
		}

		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}

else if(PlanSpeed>5){
		if(PlanSpeed>0){
			if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed<0){
			if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 += PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 -= PlanSpeed*(1500.0*50.0/1000000.0);
	}

	}else{
	leftdone=1;
	}

	 	Desired_Position_Motor1=(int)(temp_Desired_Position_Motor1*211.974522293); //Pulses per wheel round:512*39=19968 -> ||Desired_Position_Motor1=(int)(0.9*(EndPos*211.974522293)+(0.05*testsum*211.974522293));//
        Desired_Position_Motor2=(int)(temp_Desired_Position_Motor2*211.974522293);



	}
}


void UDOO::rotate_Right(){
	if(rot_tick>=1500){
		rot_tick=0;

    MmPos1=Actual_Pos_Motor1*0.004717548;//Position converted from pulses to mm
    MmPos2=Actual_Pos_Motor2*0.004717548;

temp_Desired_Position_Motor1 = MmPos1;
temp_Desired_Position_Motor2 = MmPos2;

BreakPos=(PlanSpeed*PlanSpeed)/(2*MaxAcc);


	if((rotdist_right-temp_Desired_Position_Motor2-BreakPos)>0){
		if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);//Increase PlanSpeed with maximum acceleration multiplied by sampling time for Pos_Planner
		}

		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}

else if(PlanSpeed>5){
		if(PlanSpeed>0){
			if(PlanSpeed>-MaxSpeed){
			PlanSpeed -= MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}else if(PlanSpeed<0){
			if(PlanSpeed<MaxSpeed){
			PlanSpeed += MaxAcc*(1500.0*50.0/1000000.0);
		}
		temp_Desired_Position_Motor1 -= PlanSpeed*(1500.0*50.0/1000000.0);
		temp_Desired_Position_Motor2 += PlanSpeed*(1500.0*50.0/1000000.0);
	}

	}else{
	ritdone=1;
	}

	 	Desired_Position_Motor1=(int)(temp_Desired_Position_Motor1*211.974522293); //Pulses per wheel round:512*39=19968 -> ||Desired_Position_Motor1=(int)(0.9*(EndPos*211.974522293)+(0.05*testsum*211.974522293));//
        Desired_Position_Motor2=(int)(temp_Desired_Position_Motor2*211.974522293);



	}
}
void UDOO::Position_Control_Loop(void) {
  if (pos_tick>=2000) {
	pos_tick=0;
   	Next_Position_Control_Loop = Next_Position_Control_Loop + Position_Loop_Interval;

	Actual_Pos_Motor1 = ENCODER_MOTOR_1;
	Actual_Pos_Motor2 = ENCODER_MOTOR_2;



	Desired_Speed_Motor1 = 0.1*(Desired_Position_Motor1 - Actual_Pos_Motor1);
	Desired_Speed_Motor2 = 0.1*(Desired_Position_Motor2 - Actual_Pos_Motor2);
  }
}
