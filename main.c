
#include "stm8s.h"
#include "pid_controller.h"

uint16_t read_head_sensor, hien_nhiet, count, set_temp, check_temp=0, adc0, adc100;


#define HEAT_MAX_SET 600
#define HEAT_MIN_SET 200

#define CALIB_SET_MAX 500
#define CALIB_SET_MIN 99

volatile uint32_t f_cpu;
volatile uint32_t time_keeper=0;
uint16_t check_temp, nhiet_do;
volatile uint8_t dem=0,  head_duty=0, check_encoder=0;
uint8_t  mode_calib=0, scan_temp=0, off_mode=0;
long value_read_adc=0, result_scan_adc=0;
uint16_t result_adc=0;
uint16_t adc0=266, adc100=392 ;
float heat_sensor_float=0;
int16_t calib_value_b=0;
uint16_t time_out=0, check_set_temp=0;

PIDControl pid;

static void clk_config_16MHz_hsi(void)
{       

        CLK_DeInit();
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
	CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
}

void delay_using_timer4_init(void)
{
	TIM4_TimeBaseInit(TIM4_PRESCALER_16,124);//1ms if fMaster=16Mhz
	TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);

	enableInterrupts();
	TIM4_Cmd(DISABLE);
}

void delay_isr(void)
{
	if(TIM4_GetITStatus(TIM4_IT_UPDATE)==SET)
	{
		if(time_keeper!=0)
		{
			time_keeper--;
		}
		else
		{
			/* Disable Timer to reduce power consumption */
			TIM4->CR1 &= (uint8_t)(~TIM4_CR1_CEN);
		}
		TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
	}
}


void delay_ms(uint32_t time)
{
	time_keeper=time;

	/* Reset Counter Register value */
	TIM4->CNTR = (uint8_t)(0);

	/* Enable Timer */
	TIM4->CR1 |= TIM4_CR1_CEN;

	while(time_keeper);
}


static void ADC_Config()
{
  /*  Init GPIO for ADC1 */
  GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
  
  /* De-Init ADC peripheral*/
  //ADC1_DeInit();

  /* Init ADC1 peripheral */
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_0, ADC1_PRESSEL_FCPU_D2, \
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,\
            DISABLE);
  /*Start Conversion */
  ADC1_StartConversion();
}

static void TIM2_Config(void)
{
 	TIM2_TimeBaseInit(TIM2_PRESCALER_32,50);//0.2ms if fMaster=16Mhz
	TIM2_ClearFlag(TIM2_FLAG_UPDATE);
	TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
	enableInterrupts();
	TIM2_Cmd(ENABLE);
}

unsigned int read_adc()
{
	uint32_t result_adc=0;
	for(uint16_t i=0; i<500; i++)
	{
		result_adc+=ADC1_GetConversionValue();
	}
	result_adc= (result_adc/500);
	return result_adc;
}

void quet_led(uint8_t number)
  {     
    switch (number)
      {
      case 0:
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 1: //
        GPIO_WriteHigh(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteHigh(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 2: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteHigh(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 3: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 4: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 5: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 6: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 7: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteHigh(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteHigh(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 8: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E 
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      case 9: //
        GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteLow(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteLow(GPIOC, GPIO_PIN_6); //C
        GPIO_WriteLow(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        GPIO_WriteHigh(GPIOC, GPIO_PIN_5); //DP
        break;
      }
  }

void OFF_Mode()
  {
    
    GPIO_WriteLow(GPIOD, GPIO_PIN_4);
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_2); //DIG1 , hien O
    quet_led(0);
    delay_ms(20);
    
    GPIO_WriteLow(GPIOD, GPIO_PIN_2);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_3); // DIG2
    GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
    GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
    GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
    GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E
    GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
    GPIO_WriteHigh(GPIOE, GPIO_PIN_5); //B
    GPIO_WriteHigh(GPIOC, GPIO_PIN_6); //C
    delay_ms(15);
    
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_4);  // DIG3
    GPIO_WriteLow(GPIOC, GPIO_PIN_1); //F
    GPIO_WriteLow(GPIOC, GPIO_PIN_2); //A
    GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
    GPIO_WriteLow(GPIOC, GPIO_PIN_3); //E
    GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
    GPIO_WriteHigh(GPIOE, GPIO_PIN_5); //B
    GPIO_WriteHigh(GPIOC, GPIO_PIN_6); //C
    delay_ms(10);
    
  }
  
  
void wirte_flash_settemp() // luu nhiet do
   {
     //FLASH_DeInit();
     FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);
     FLASH_Unlock(FLASH_MEMTYPE_DATA);

     FLASH_EraseByte(0x4000);
     FLASH_ProgramByte(0x4000,(set_temp/100));
     
     FLASH_EraseByte(0x4001);
     FLASH_ProgramByte(0x4001,(set_temp%100/10));
     
     FLASH_EraseByte(0x4002);
     FLASH_ProgramByte(0x4002,(set_temp%10));

     FLASH_Lock(FLASH_MEMTYPE_PROG);
   }        
 
void read_flash_settemp() // doc nhiet do da luu trong flash
   {
     uint8_t tram, chuc, donvi;
     set_temp=0; 
     tram=FLASH_ReadByte(0x4000);
     chuc=FLASH_ReadByte(0x4001);
     donvi=FLASH_ReadByte(0x4002);
     set_temp=tram*100+chuc*10+donvi;
   }



void hienthi(uint16_t gia_tri)
 {
    GPIO_WriteLow(GPIOD, GPIO_PIN_4);
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_2); //DIG1 
    quet_led((gia_tri/100));
    delay_ms(20);

    GPIO_WriteLow(GPIOD, GPIO_PIN_2);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_3); // DIG2
    quet_led((gia_tri%100/10));
    delay_ms(15);
    
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_4);  // DIG3
    quet_led((gia_tri%10));
    if((check_encoder==1)&&(mode_calib==0)) GPIO_WriteLow(GPIOC, GPIO_PIN_5); //DP
    delay_ms(10);
 }

void hien_thi_calib(int16_t calib)
 {
    int16_t a=0;
    
    GPIO_WriteLow(GPIOD, GPIO_PIN_4);
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteHigh(GPIOD, GPIO_PIN_2); //DIG1 
    if(calib<0)
      {
        // bao gia tri am
        GPIO_WriteHigh(GPIOC, GPIO_PIN_6); //C 
        GPIO_WriteHigh(GPIOC, GPIO_PIN_2); //A
        GPIO_WriteHigh(GPIOE, GPIO_PIN_5); //B
        GPIO_WriteHigh(GPIOC, GPIO_PIN_1); //F
        GPIO_WriteHigh(GPIOC, GPIO_PIN_4); //D
        GPIO_WriteLow(GPIOC, GPIO_PIN_7); //G
        GPIO_WriteHigh(GPIOC, GPIO_PIN_3); //E
        a=-calib;
        delay_ms(20);
        GPIO_WriteLow(GPIOD, GPIO_PIN_2);
        GPIO_WriteHigh(GPIOD, GPIO_PIN_3); // DIG2
        quet_led((a/10));
        delay_ms(15);
    
        GPIO_WriteLow(GPIOD, GPIO_PIN_3);
        GPIO_WriteHigh(GPIOD, GPIO_PIN_4);  // DIG3
        quet_led((a%10));
        delay_ms(10);
      }
    else 
      {
        
        a=calib;
        hienthi(calib);
        GPIO_WriteReverse(GPIOC, GPIO_PIN_5); //DP
      }
      


 }


void writeflash_calib(int16_t temp)
  {
     uint16_t calib_temp=0;
     //FLASH_DeInit();
     FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);
     FLASH_Unlock(FLASH_MEMTYPE_DATA);
     FLASH_EraseByte(0x4007);
     if(temp<0)
     {
       FLASH_ProgramByte(0x4007,10);
       calib_temp = -temp;
     }
     else
     {
       FLASH_ProgramByte(0x4007,20);
       calib_temp = temp;
     }
     /// save calib
     FLASH_EraseByte(0x4003);
     FLASH_ProgramByte(0x4003,(calib_temp/100));    
     FLASH_EraseByte(0x4004);
     FLASH_ProgramByte(0x4004,(calib_temp%100/10));    
     FLASH_EraseByte(0x4005);
     FLASH_ProgramByte(0x4005,(calib_temp%10));
     //FLASH_Lock(FLASH_MEMTYPE_PROG);
  }



void read_flash_calib() // doc nhiet do da luu trong flash
   {
     uint8_t adc0tram, adc0chuc, adc0donvi, dau;
     uint16_t calib_value;
     dau = FLASH_ReadByte(0x4007);
     adc0tram=FLASH_ReadByte(0x4003);
     adc0chuc=FLASH_ReadByte(0x4004);
     adc0donvi=FLASH_ReadByte(0x4005);
     calib_value=adc0tram*100+adc0chuc*10+adc0donvi;
     if(dau == 20) calib_value_b = calib_value;
     else calib_value_b = -calib_value;
       
   }





void check_calib()
{
    if((GPIO_ReadInputData(GPIOB) & GPIO_PIN_7)==0x00)
     {
       mode_calib=1; 
       GPIO_WriteLow(GPIOC, GPIO_PIN_7); // thong bao che do set_calib 
       delay_ms(500);
       for(char i=0; i<8; i++)
       {
       GPIO_WriteReverse(GPIOD, GPIO_PIN_0);
       delay_ms(500);
       }
       while((GPIO_ReadInputData(GPIOB) & GPIO_PIN_7)==0x00);
     }
}

void Set_Heat()
  {
     if(off_mode)  // vao che do ngu (off)
     {
       off_mode=0;
     }
     if(mode_calib) //vao che do can chinh (calib) nhiet do tram han
     {
       if((GPIO_ReadInputData(GPIOD) & GPIO_PIN_5)==0x00)
	  {   
            check_encoder=1;
            calib_value_b++; 
            if(calib_value_b>CALIB_SET_MAX) calib_value_b=-CALIB_SET_MIN;
	  }
     
       if((GPIO_ReadInputData(GPIOD) & GPIO_PIN_6)==0x00)
	  {      
            check_encoder=1;
            calib_value_b--;
            if(calib_value_b<-CALIB_SET_MIN) calib_value_b=CALIB_SET_MAX;
	  }
     }
     else // vao che do set nhiet do mong muon
     {
       if((GPIO_ReadInputData(GPIOD) & GPIO_PIN_5)==0x00)
	  {   
            if(set_temp>HEAT_MAX_SET) set_temp=HEAT_MIN_SET;
            check_encoder=1;
            set_temp++;   
	  }
     
       if((GPIO_ReadInputData(GPIOD) & GPIO_PIN_6)==0x00)
	  {      
            if(set_temp<HEAT_MIN_SET) set_temp=HEAT_MAX_SET;
            check_encoder=1;
            set_temp--;
	  }
     }
  }

 

void head_control()
    {
      if(TIM2_GetITStatus(TIM2_IT_UPDATE)==SET)
     {
      dem++;      
      if(dem<=head_duty) GPIO_WriteHigh(GPIOB, GPIO_PIN_3); else GPIO_WriteLow(GPIOB, GPIO_PIN_3); // BAM XUNG FET
      if(dem==255) dem=0;
      TIM2_ClearITPendingBit(TIM2_IT_UPDATE);
     }
     
    }

void check_time_out()
{
  while((GPIO_ReadInputData(GPIOB) & GPIO_PIN_7)==0x00)
  {
    GPIO_WriteLow(GPIOD, GPIO_PIN_4);
    GPIO_WriteLow(GPIOD, GPIO_PIN_3);
    GPIO_WriteLow(GPIOD, GPIO_PIN_2); //DIG1 
    time_out++;
    delay_ms(100);
    if(time_out==15) 
    {
     GPIO_WriteHigh(GPIOD, GPIO_PIN_0);  //SPK
     delay_ms(1000);
     GPIO_WriteLow(GPIOD, GPIO_PIN_0);  //SPK
     delay_ms(500);
     off_mode=1;
    }
  }
}

void Option_Bytes_Configuration()
{
  unsigned short temp;
  //unsigned char value_optbyte;

  //----------------------------------------------------------------------------
  //Resets the FLASH peripheral registers to their default reset values
  FLASH_DeInit();
  //----------------------------------------------------------------------------
  //Unlock flash data eeprom memory
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  //Wait until Data EEPROM area unlocked flag is set
  //while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {};
  //Define flash programming Time
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  //----------------------------------------------------------------------------
  //OPT0: Memory Read-Out Protection (ROP)
  temp = FLASH_ReadOptionByte(0x4800);
  //value_optbyte = temp>>8;
  if(temp != 0xAA)
  {
  FLASH_ProgramOptionByte(0x4800, 0xAA);
  }
  //FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void control_soldering()
{
            heat_sensor_float =  read_adc()*0.619009585+calib_value_b;
            read_head_sensor = (uint16_t)heat_sensor_float;
            //PIDTuningKpSet (&pid,5120);
            //PIDTuningKiSet (&pid,512);
            //PIDTuningKdSet (&pid,768);
            PIDSetpointSet(&pid,set_temp);
            PIDInputSet(&pid,read_head_sensor);
            PIDCompute(&pid);
            head_duty=PIDOutputGet(&pid); // xuat xung pwm
}



void main(void)
{
  uint8_t buzz=0;
  
  clk_config_16MHz_hsi();
  // NUT NHAN ENCODER
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_IN_PU_IT);        //LEFT
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_IT);        //RIGH
  GPIO_Init(GPIOB, GPIO_PIN_7, GPIO_MODE_IN_PU_IT);        //OK
  GPIO_Init(GPIOB, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);        //calib
  ///////////////////////////////////////////////////////////////////
  
  // LED 7SEG
  GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST); //a
  GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); //b
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST); //c
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST); //d
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST); //e
  GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_HIGH_FAST); //f
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_HIGH_FAST); //g
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); //DP
  
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST); //A1
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST); //A2
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST); //A3
  /////////////////////////////////////////////////////////////
  
  // COI CHIP
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST); //SPK
  ///////////////////////////////////////////////////////////////
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
  /////////////////////////////////////////////////////////
  GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);  //PWM
  delay_using_timer4_init();
  TIM2_Config();
  ADC_Config();  //SENSOR HAN
  Option_Bytes_Configuration();
  enableInterrupts();
  check_calib();
  if(mode_calib==0)
    {
     hienthi(888);
     GPIO_WriteHigh(GPIOD, GPIO_PIN_0);  //SPK
     delay_ms(1000);
     GPIO_WriteLow(GPIOD, GPIO_PIN_0);  //SPK
     delay_ms(500);
    }
  read_flash_calib();
  read_flash_settemp();
  //PIDInit(&pid,256,54,150,0.05,0,255,AUTOMATIC,DIRECT);
  PIDInit(&pid,5120,512,768,0.05,0,255,AUTOMATIC,DIRECT);
  while (1)
  {
   if(mode_calib)
    {
      hien_thi_calib(calib_value_b);
    }
   else
    {
      if(off_mode) // che do OFF, tram han tat mosfet, cho den khi co tin hieu bam nut hoac van nut
       {
        OFF_Mode();
        head_duty=0;
       }
      else
       {
         time_out=0;
         if(check_encoder==1) // vao che do set nhiet do
           {
            check_temp=0;
            check_time_out();  // neu qua time out, mach se chuyen qua che do OFF
            hienthi(set_temp);
            control_soldering();
            // trong che do calib, van dieu khien muc nhiet do da set truoc do
            //result_adc = read_adc();
            /*
            heat_sensor_float =  read_adc()*0.619009585+calib_value_b;
            read_head_sensor = (uint16_t)heat_sensor_float;
            PIDTuningKpSet (&pid,5120);
            PIDTuningKiSet (&pid,512);
            PIDTuningKdSet (&pid,768);
            PIDSetpointSet(&pid,set_temp);
            PIDInputSet(&pid,read_head_sensor);
            PIDCompute(&pid);
            head_duty=PIDOutputGet(&pid); // xuat xung pwm
            */
           }
         else
           {
            count++;
            buzz--;
            /*
            result_adc = read_adc();
            heat_sensor_float =  result_adc*0.619009585+calib_value_b;
            read_head_sensor = (uint16_t)heat_sensor_float;
            //PIDTuningKpSet (&pid,256);
            //PIDTuningKiSet (&pid,54);
            //PIDTuningKdSet (&pid,150);
            PIDTuningKpSet (&pid,5120);
            PIDTuningKiSet (&pid,512);
            PIDTuningKdSet (&pid,768);
            PIDSetpointSet(&pid,set_temp);
            PIDInputSet(&pid,read_head_sensor);
            PIDCompute(&pid);
            head_duty=PIDOutputGet(&pid); // xuat xung pwm
            */
            control_soldering();
            if(check_temp==0)
              { 
               count=0;
               hien_nhiet=read_head_sensor;
               hienthi(hien_nhiet);
              
           /*
           check_temp=abs(set_temp-read_head_sensor);
           if(check_temp>10) check_spk=1;
           if((check_spk==1)&&(check_temp==0)&&(buzz<50)) 
            {
            buzz = 100;
            GPIO_WriteHigh(GPIOD, GPIO_PIN_0);  //SPK
            delay_ms(500);
            GPIO_WriteLow(GPIOD, GPIO_PIN_0);  //SPK
            check_spk=0;
            }
          */
             check_set_temp= abs(set_temp-read_head_sensor);
             if(check_set_temp<4)
              {
               check_temp=1;
               GPIO_WriteHigh(GPIOD, GPIO_PIN_0);  //SPK
               delay_ms(500);
               GPIO_WriteLow(GPIOD, GPIO_PIN_0);  //SPK
              }
             }
          else if(check_temp==1)
          {
            hienthi(set_temp);
            if(set_temp>read_head_sensor)
            {
            check_set_temp= set_temp-read_head_sensor;
            if (check_set_temp>10)
            {
            check_temp=0;
            } 
            } 
            
          }
         }
       }
  }

  
  }
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
