/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_16K.h"
#include <stdlib.h>

/******************************************************************************/
#define MAXTASKS 		(16U)
static unsigned char timers[MAXTASKS];
#define _TaskStart_    static unsigned char _lc=0U; switch(_lc){case 0U:  
#define _TaskEnd_     break; default:break;}  _lc=0U;  return 255U;

//Must under 255
#define DelayTask(tickets)   	_lc=(unsigned char)(__LINE__)%255U+1U;  if(_lc){return (tickets);}  break; case ((unsigned char)(__LINE__)%255U)+1U:
#define Entry(void)   		_lc=(unsigned char)(__LINE__)%255U+1U;  if(_lc){return (1);}  break; case ((unsigned char)(__LINE__)%255U)+1U:

//#define WaitUntil(condition)  do {_lc=(__LINE__+((__LINE__%256)==0))%256; }while(0); case (__LINE__+((__LINE__%256)==0))%256: if(!(condition)){return 0 ;} 
//#define WaitUntil(CondExpr,Cyc)     do { WaitX(Cyc);  }while( !(CondExpr) )

#define StartTask(TaskName,TaskID)  do {if (timers[(TaskID)]==0U){ timers[(TaskID)]=(TaskName)(); } } while(0)  
#define RunTask(TaskName,TaskID) { if (timers[(TaskID)]==0U) {timers[(TaskID)]=(TaskName)(); continue;} }	//alwasy execute task before this

#define CallSub(SubTaskName) do {unsigned char currdt; _lc=(unsigned char)(__LINE__)%255U+1U; if(_lc) {return 0U;} break; case (unsigned char)(__LINE__)%255U+1U:  currdt=(SubTaskName)(); if(currdt!=255U) {return currdt;}} while(0);
#define InitTasks() do {unsigned char i; for(i=MAXTASKS;i>0U ;i--) {timers[i-1U]=0U;} } while(0)

#define SEM unsigned int
#define InitSem(sem) (sem)=0;
#define WaitSem(sem) do{ (sem)=1; DelayTask(0); if ((sem)>0) return 1;} while(0);
/*tickets Max : 0xFFFE*/
#define WaitSemX(sem,tickets)  do { (sem)=(tickets)+1; DelayTask(0); if((sem)>1){ (sem)--;  return 1;} } while(0);
#define SendSem(sem)  do {(sem)=0;} while(0);
//static unsigned long t1ms;

// TODO: Insert below fuction in 1ms TIMx interrupt or Systick interrupt
#define UpdateTimers() do { unsigned char i;  for(i=MAXTASKS;i>0U ;i--){      if(timers[i-1U]!=0U){          if(timers[i-1U]!=255U) { timers[i-1U]--;} } }} while(0)

typedef struct
{
	unsigned char (*handler)();
 	
}Task_TypeDef;

/******************************************************************************/

typedef enum{
	TARGET_CH0 = 0 ,
	TARGET_CH1 ,
	TARGET_CH2 ,
	TARGET_CH3 ,	
	
	TARGET_CH4 ,
	TARGET_CH5 ,
	TARGET_CH6 ,
	TARGET_CH7 ,

	TARGET_CH_DEFAULT	
}Channel_TypeDef;

typedef enum
{
	ADC_DataState_AVERAGE = 0 ,
	ADC_DataState_MMA , 
	ADC_DataState_DROP , 
	
	ADC_DataState_DEFAULT 	
}ADC_DataState_TypeDef;


//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

#define LED_REVERSE(x)							(100-x)			// because lED in EVM schematic , need to reverse level

#define TIMER_LOG_MS							(1000ul)
//#define ADC_SAMPLETIME_MS						(20ul)
#define GPIO_TOGGLE_MS							(500ul)

#define ADC_RESOLUTION							(4096ul)
//#define ADC_REF_VOLTAGE							(3300ul)	//(float)(3.3f)

#define ADC_MAX_TARGET							(4095ul)	//(float)(2.612f)
#define ADC_MIN_TARGET							(0ul)	//(float)(0.423f)

#define DUTY_MAX								(100ul)
#define DUTY_MIN								(1ul)
//#define ADC_CONVERT_TARGET						(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 

#define ADC_SAMPLE_COUNT 						(8ul)			// 8
#define ADC_SAMPLE_POWER 						(3ul)			//(5)	 	// 3	,// 2 ^ ?
#define ADC_SAMPLE_DROP 						(4ul)

#define CUSTOM_INPUT_VOLT_MAX(VREF)			(VREF)			//(3300ul)
#define CUSTOM_INPUT_VOLT_MIN					(0)	//(600ul)

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 

//#define ENABLE_ADC_MMA
#define ENABLE_ADC_DROP_AVG

uint8_t 	u8TH0_Tmp = 0;
uint8_t 	u8TL0_Tmp = 0;

//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

double  Bandgap_Voltage,AVdd,Bandgap_Value;      //please always use "double" mode for this
unsigned char xdata ADCdataVBGH, ADCdataVBGL;

uint16_t adc_target = 0;
unsigned long int adc_sum_target = 0;
uint16_t adc_data = 0;
uint16_t adc_convert_target = 0;
uint16_t adc_ref_voltage = 0;

uint16_t seed = 0;

ADC_DataState_TypeDef ADCDataState = ADC_DataState_DEFAULT;

unsigned long int tickstart = 0;
unsigned long int uwTick = 0;
	
typedef enum{
	flag_LED = 0 ,
	flag_PID_Finish ,
	
	flag_DEFAULT	
}Flag_Index;

#define ABS(X)  									((X) > 0 ? (X) : -(X))

uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

#define PID_INCREMENTAL 
//#define PID_ANTI_WINDUP

unsigned long int GetTick(void);

struct t_pid
{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;	
	float Kp,Ki,Kd;
	
	#if defined (PID_INCREMENTAL)
	float err_next;
	#endif

	#if defined (PID_ANTI_WINDUP)
	float voltage;
	float integral;
	float umax;
	float umin;
	#endif
}pid;

void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}

void GPIO_Toggle(void)
{
    P12 ^= 1 ;
}

void GPIO_Init(void)
{
    P12_PUSHPULL_MODE;
}

void ADC_ReadAVdd(void)
{
    UINT8 BandgapHigh,BandgapLow,BandgapMark;
    double bgvalue;

/*Read bandgap value */	
    set_CHPCON_IAPEN;
    IAPCN = READ_UID;
    IAPAL = 0x0d;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapLow = IAPFD;
    BandgapMark = BandgapLow&0xF0;
    BandgapLow = BandgapLow&0x0F;
    IAPAL = 0x0C;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapHigh = IAPFD;
    Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
    Bandgap_Voltage= Bandgap_Value*3/4;
    clr_CHPCON_IAPEN;

/* ADC Low speed initial*/  
    ENABLE_ADC_BANDGAP;
    ADCCON1|=0x30;            /* clock divider */
    ADCCON2|=0x0E;            /* AQT time */
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	
/*start bandgap ADC */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                                
    while(ADCF == 0);
    ADCdataVBGH = ADCRH;
    ADCdataVBGL = ADCRL;
	
/* to convert VDD value */
    bgvalue = (ADCRH<<4) + ADCRL;
    AVdd = (0x1000/bgvalue)*Bandgap_Voltage;

//    printf ("\r\n BG Voltage = %e\r\n", Bandgap_Voltage); 
//    printf ("\r\n VDD voltage = %e\r\n", AVdd); 	
}

uint16_t ADC_DropAndAverage (uint8_t drop , uint8_t avg)
{
	uint8_t n = 0;

	switch(ADCDataState)
	{
		case ADC_DataState_DROP:
			for ( n = 0 ; n < drop ; n++)
			{
				while(ADCF);
			 	adc_data = 0;					
				set_ADCCON0_ADCS; //after convert , trigger again	
			}				
			ADCDataState = ADC_DataState_AVERAGE;

			break;
			
		case ADC_DataState_AVERAGE:
			for ( n = 0 ; n < avg ; n++)
			{
				while(ADCF);
				adc_sum_target += adc_data;					
				set_ADCCON0_ADCS; //after convert , trigger again
			}
			adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;
			break;				
	}
	
	adc_sum_target = 0;

	return adc_target;
}

uint16_t ADC_ModifiedMovingAverage (void)
{
	static uint16_t cnt = 0;
		
	switch(ADCDataState)
	{
		case ADC_DataState_AVERAGE:
			while(ADCF);
			adc_sum_target += adc_data;
			if (cnt++ >= (ADC_SAMPLE_COUNT-1))
			{
				cnt = 0;
				adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
				ADCDataState = ADC_DataState_MMA;
			}			
			break;
			
		case ADC_DataState_MMA:
			while(ADCF);
			adc_sum_target -=  adc_target;
			adc_sum_target += adc_data;
			adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;

			break;				
	}
	

	return adc_target;
}


void ADC_Parameter_Initial(void)
{
	#if defined (ENABLE_ADC_MMA)
	ADCDataState = ADC_DataState_AVERAGE;
	#endif
		
	#if defined (ENABLE_ADC_DROP_AVG)
	ADCDataState = ADC_DataState_DROP;
	#endif
		
	adc_sum_target = 0;
	adc_target = 0;

}

uint16_t ADC_To_Voltage(uint16_t adc_value)
{
	uint16_t volt = 0;

	volt = (AVdd*adc_value)/ADC_DIGITAL_SCALE();
	
//	send_UARTString("adc_value:");	
//	send_UARTASCII(adc_value);
//	send_UARTString(",volt:");
//	send_UARTASCII(volt);
//	send_UARTString("mv,AVdd:");
//	send_UARTASCII(AVdd);	
//	send_UARTString("mv\r\n");

	return volt;	
}

uint16_t ADC_To_Duty(uint16_t adc_value)
{
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;
	uint16_t volt_max = CUSTOM_INPUT_VOLT_MAX(AVdd);//CUSTOM_INPUT_VOLT_MAX(0);
	uint16_t volt_min = CUSTOM_INPUT_VOLT_MIN;
	uint16_t duty = 0;
	uint16_t adc_target = 0;	
	uint16_t interval = DUTY_MAX - DUTY_MIN + 1;	
	adc_ref_voltage = AVdd;
	
	adc_max = (ADC_RESOLUTION * volt_max)/adc_ref_voltage ;
	adc_min = (ADC_RESOLUTION * volt_min)/adc_ref_voltage ;	
	
	adc_target = (adc_value <= adc_min) ? (adc_min) : (adc_value) ;
	adc_target = (adc_target >= adc_max) ? (adc_max) : (adc_target) ;

	duty = (float)(adc_target - adc_min)*interval/(adc_max - adc_min) + 1;
	duty = (duty >= DUTY_MAX) ? (DUTY_MAX) : (duty) ;
	
//	send_UARTString("adc_value:");	
//	send_UARTASCII(adc_value);
//	send_UARTString(",adc_min:");
//	send_UARTASCII(adc_min);
//	send_UARTString(",adc_max:");
//	send_UARTASCII(adc_max);
//	send_UARTString(",adc_target:");
//	send_UARTASCII(adc_target);
//	send_UARTString(",duty:");
//	send_UARTASCII(duty);
//	send_UARTString("\r\n");

	return duty;	
}


uint16_t ADC_ConvertChannel(void)
{
	volatile uint16_t adc_value = 0;
	volatile uint16_t duty_value = 0;
	adc_ref_voltage = AVdd;
	
	adc_convert_target = (ADC_MIN_TARGET*ADC_RESOLUTION/adc_ref_voltage);

	#if defined (ENABLE_ADC_DROP_AVG)
	adc_value = ADC_DropAndAverage(ADC_SAMPLE_DROP,ADC_SAMPLE_COUNT);
//	send_UARTString("adc_value (DropAndAverage) :");	
//	send_UARTASCII(adc_value);
//	send_UARTString("\r\n");

	adc_value = (adc_value <= adc_convert_target) ? (adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	duty_value = ADC_To_Duty(adc_value);	
	ADC_To_Voltage(adc_value);
	
	#endif
	
	#if defined (ENABLE_ADC_MMA)
	adc_value = ADC_ModifiedMovingAverage();
//	send_UARTString("adc_value (MMA) :");	
//	send_UARTASCII(adc_value);
//	send_UARTString("\r\n");
	
	adc_value = (adc_value <= adc_convert_target) ? (adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	duty_value = ADC_To_Duty(adc_value);	
	ADC_To_Voltage(adc_value);

	set_ADCCON0_ADCS; //after convert , trigger again
	#endif
	
	return adc_value;
}

void ADC_ISR(void) interrupt 11          // Vector @  0x5B
{	
    _push_(SFRS);

//	adc_data = ((ADCRH<<4) + ADCRL);	
	adc_data = (((ADCRH<<4) + ADCRL)>>1)<<1;

//	send_UARTString("ADC_ISR :");	
//	send_UARTASCII(adc_data);
//	send_UARTString("\r\n");

    clr_ADCCON0_ADCF; //clear ADC interrupt flag

     _pop_(SFRS);   
}

void ADC_InitChannel(uint8_t CH)
{
	ADC_ReadAVdd();

	switch(CH)
	{
		case TARGET_CH0: 
		    ENABLE_ADC_AIN0;
			break;

		case TARGET_CH1: 
		    ENABLE_ADC_AIN1;
			break;

		case TARGET_CH2: 
		    ENABLE_ADC_AIN2;
			break;

		case TARGET_CH3: 
		    ENABLE_ADC_AIN3;
			break;

		case TARGET_CH4: 
		    ENABLE_ADC_AIN4;
			break;

		case TARGET_CH5: 
		    ENABLE_ADC_AIN5;
			break;

		case TARGET_CH6: 
		    ENABLE_ADC_AIN6;
			break;

		case TARGET_CH7: 
		    ENABLE_ADC_AIN7;
			break;		
		
	}

  /* ADC Low speed initial*/  
    ADCCON1|=0X30;            /* clock divider */
    ADCCON2|=0X0E;            /* AQT time */

	#if 0
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	#else
    AUXR1 &= ~SET_BIT4;			//high speed , 500k sps
	#endif

	clr_ADCCON0_ADCF;
	set_ADCCON0_ADCS;                  // ADC start trig signal

	#if 0	//polling
	while(ADCF == 0);
	#else	// Enable ADC interrupt (if use interrupt)
    set_IE_EADC;                        
    ENABLE_GLOBAL_INTERRUPT;
	#endif

	ADC_Parameter_Initial();

//	return ((ADCRH<<4) + ADCRL);

}

void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void UART0_Init(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}

void PID_init(float Kp , float Ki , float Kd)
{
    pid.SetSpeed = 0.0;
    pid.ActualSpeed = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;	

	#if defined (PID_INCREMENTAL)	
	pid.err_next = 0.0;
	#endif 

	#if defined (PID_ANTI_WINDUP)	
	pid.voltage = 0.0;
	pid.integral = 0.0;
	pid.umax = 400;
	pid.umin =-200; 
	#endif 
}

float PID_realize(float speed)
{
	#if defined (PID_INCREMENTAL)
	float incrementSpeed;	
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	
    incrementSpeed = pid.Kp*(pid.err-pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2*pid.err_next + pid.err_last);
    pid.ActualSpeed += incrementSpeed;
    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
	#endif

	#if defined (PID_ANTI_WINDUP)
	int index;
	pid.SetSpeed=speed;
	
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	
	if (pid.ActualSpeed>pid.umax)
	{
		if(ABS(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pid.err<0)
			{
				pid.integral+=pid.err;
			}
		}
	}
	else if (pid.ActualSpeed<pid.umin)
	{
		if(ABS(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}
	}
	else
	{
		if(ABS(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			pid.integral+=pid.err;
		}
	}
	
	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	#endif
	
	return pid.ActualSpeed;
}

void PID_calculate(int timer , float target)
{
	#if 1
	int tmp = timer ;
    static int cnt = 0;
  	int approach = 0;
	static uint16_t current = 0;
	static uint16_t previous = 0;
	uint16_t convergence = 0;
	uint16_t res = 0;

	if (previous == (uint16_t)target)
	{
//		send_UARTString("target same last data , skip flow\r\n");
		return;
	}

	current = (uint16_t)target;
	convergence = current % 10;

	set_flag(flag_PID_Finish , Disable);

	while(!is_flag_set(flag_PID_Finish))
    {
		approach = (uint16_t) PID_realize(current);		

		res = (current >= previous ) ? (current - previous) : (previous - current) ;
		cnt = (res <= convergence) ? (cnt+1) : (cnt) ;

//		send_UARTString("cnt : ");	
//		send_UARTASCII(cnt);
//		send_UARTString(",convergence: ");
//		send_UARTASCII(convergence);	
		send_UARTString(",current: ");
		send_UARTASCII(current);	
//		send_UARTString(",previous: ");
//		send_UARTASCII(previous);			
//		send_UARTString(", time :");
//		send_UARTASCII(GetTick() - tickstart);
		send_UARTString(", approach : ");
		send_UARTASCII(approach);
//		send_UARTString(", target :");
//		send_UARTASCII((uint16_t)target);
		send_UARTString("\r\n");

		res = (approach >= current ) ? (approach - current) : (current - approach) ; 
		if (res <= 1)
		{   		
			send_UARTString("\r\n");
//			send_UARTString(", time :");
//			send_UARTASCII(GetTick() - tickstart);
			send_UARTString(",FINISH !");
			send_UARTString("\r\n\r\n");
			
			set_flag(flag_PID_Finish , Enable);
			cnt = 0;
			
			break;
		}		
    };

	previous = (int)target;
	
	#else
	
    int cnt = 0;
  	float speed = 0;

	while(cnt<timer)
    {
        speed = PID_realize(target);
        printf("%f\n",speed);
        cnt++;
    }	
	#endif
}

unsigned char task_LED(void)
{
	static int i;  
	
	_TaskStart_

	while(1)
	{
		Entry();	
		
		//insert application		
		for(i=0;i<10;i++)
		{   
			DelayTask(200U);
		}
		send_UARTString("LED:2000ms\r\n");
		GPIO_Toggle();
	}
	_TaskEnd_
}

unsigned char task_GenerateSeed(void)
{

	_TaskStart_

	while (1)
	{
		Entry();
		
		//insert application	
		srand(ADC_ConvertChannel());//use ADC as random seed
		seed = (rand()%(0x1000-1)) + 1;	//((rand()+tickstart)%(0xFFFF-1)) + 1;	
	}
	
	_TaskEnd_
}


unsigned char task_PID(void)
{
//	static int i;  

	_TaskStart_

	#if 1	//execute always
	while (1)
	{
		Entry();
		
		//insert application
		tickstart = GetTick();
		PID_calculate(NULL , seed);
	
	}
	#else	//execute once
	for(i=0;i<5;i++)
	{   
		WaitX(200U);
	}
	send_UARTString("RUN task0 (1s)\r\n");
	#endif
	
	_TaskEnd_
}


unsigned char task_logger_1000ms(void)
{
	static int i;  
	
	_TaskStart_

	#if 1	//execute always
	while (1)
	{
		Entry();

		//insert application
		for(i=0;i<10;i++)
		{   
			DelayTask(100U);
		}

		send_UARTString("logger:1000ms\r\n");	
	}
	#else	//execute once
	for(i=0;i<5;i++)
	{   
		WaitX(200U);
	}
	send_UARTString("RUN task0 (1s)\r\n");
	#endif
	
	_TaskEnd_
}

unsigned char task_Entry(void)
{
//	static int i;  
	
	_TaskStart_

	#if 1	//execute always
	while (1)
	{
		Entry();	//to entry case

		//insert application
//		DelayTask(0U);

	}
	#else	//execute once
	Entry(0U);

	//insert application
	for(i=0;i<5;i++)
	{   
		DelayTask(200U);
	}
	send_UARTString("RUN task0 (1s)\r\n");
	#endif
	
	_TaskEnd_
}

Task_TypeDef TaskList[] =
{
	{task_Entry			},		// this is the START of list marker	
		
	/**************************/		
	{task_LED 			},
	{task_GenerateSeed	},		
	{task_PID 			},
//	{task_logger_1000ms	},
		
	{NULL 				},  	// this is the END of list marker	
};

void TaskSchedulerInit(void)
{
	InitTasks();
}

void TaskSchedulerStart(void)
{	
	static uint8_t i = 0;	
	
	while(1)
	{
		StartTask(task_Entry,0);

	    for (i = 1 ; TaskList[i].handler != NULL; i++)
	    {
			RunTask(TaskList[i].handler , i);
	    }
	}
}

//unsigned long GetTaskTick(void)
//{
//	return t1ms;
//}

void TaskTick(void)
{
//	t1ms++;//not use
	UpdateTimers();
}

void IncTick(void)
{
	uwTick++;
}


unsigned long int GetTick(void)
{
	return uwTick;
}

void Timer0_IRQHandler(void)
{
//	static uint16_t LOG_TIMER = 0;
//	static uint16_t CNT_TIMER = 0;
//	static uint16_t CNT_GPIO = 0;

//	if (CNT_GPIO++ >= GPIO_TOGGLE_MS)
//	{		
//		CNT_GPIO = 0;
//		GPIO_Toggle();
//	}	

//	if (CNT_TIMER++ >= TIMER_LOG_MS)
//	{		
//		CNT_TIMER = 0;
//	}

	IncTick();

}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
	
	TaskTick();// 1ms interrupt to maintain task
}

void TIMER0_Init(void)
{
	uint16_t res = 0;

	ENABLE_TIMER0_MODE1;
	
	u8TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}

#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{

//	void (*pFun)() = SysTickConfig;	//function name = address
//	#if 1
//	(*pFun)();             
//	pFun();	//same behavior
//	#else
//	SysTickConfig();
//	#endif

    SYS_Init();

    UART0_Init();

	//P0.4 , ADC_CH5
	ADC_InitChannel(TARGET_CH5);

	//P12 , GPIO
	GPIO_Init();					
			
	TIMER0_Init();

	PID_init(0.2 , 0.015 , 0.2);

//	PID_calculate(1000 , 200.0);	

	TaskSchedulerInit();

	/* Infinite loop */
	TaskSchedulerStart();
	

}



