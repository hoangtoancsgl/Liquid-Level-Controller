/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "lcd_16x2.h"
#include "FLASH_PAGE.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// arrow character
uint8_t arrow_char[8] = {0x00, 0x04, 0x06, 0x1F, 0x06, 0x04, 0x00};
unsigned int Pump_status=1;
unsigned int prePump_status;
float uplevel=1;
float downlevel=2;
float level=3;
float prelevel;
unsigned int Data[5] = {1, 2, 3, 4, 0xFFFF};
unsigned int RxData[4];
volatile short cnt=0;
volatile short count=0;
volatile float test=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
 void bip()
 {
   HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);
   HAL_Delay(100);
   HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET); 
 }
 void WriteFlash()
{
    Data[0] = uplevel*4;
    Data[1] = downlevel*4;
    Data[2] = count; //level*4;
    Data[3] = Pump_status;
    Flash_Write_Data(0x801FC00, Data);
}

void ReadFlash()
{
    Flash_Read_Data(0x801FC00, RxData);
    uplevel = (float)(RxData[0])/4;
    downlevel = (float)(RxData[1])/4;
    count = RxData[2];// (float)(RxData[2])/4;
    Pump_status = (RxData[3]);
}
void Start()
{
  Lcd_clear_display();
  Lcd_gotoxy(0,0);
  Lcd_write_string("APPLIED");
  Lcd_gotoxy(5,1);
  Lcd_write_string("ELECTRONICS"); 
  HAL_Delay(3000);
  Lcd_clear_display();
}
void DisplayPara(float level, uint8_t Pumpstatus)
{
  float a=level;
  Lcd_gotoxy(0,0);
  Lcd_write_string("Level: ");
  Lcd_write_int((int)level);
  Lcd_write_string(".");
  level = level-(int)level;
  Lcd_write_int((int)10*level);
  if(a<10) 
  {
    Lcd_clear_xy(10,0);
    Lcd_gotoxy(11,0);
    Lcd_write_string("cm ");
  }
  else
  {
    Lcd_clear_xy(11,0);
    Lcd_gotoxy(12,0);
    Lcd_write_string("cm ");
  }
    
  Lcd_gotoxy(0,1);
  Lcd_write_string("Pump status: "); 
  if(Pumpstatus) Lcd_write_string("ON ");
    else Lcd_write_string("OFF");
}
uint8_t CheckSW()
{
  if(!HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)) 
  {
    HAL_Delay(20);
    if(!HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)) 
    {
      while(!HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin));
      return 1;
    }  
  }
  return 0;
}
void DisplaySetting(uint8_t list)
{
  Lcd_gotoxy(0,0);
  if(list==0) Lcd_write_custom_char(0,0,0);
    else Lcd_write_string(" ");
  Lcd_write_string("Calib");
  
  Lcd_gotoxy(11,0);
  if(list==2) Lcd_write_custom_char(11,0,0);
    else Lcd_write_string(" ");
  Lcd_write_string("Exit");
  
  Lcd_gotoxy(0,1);
  if(list==1) Lcd_write_custom_char(0,1,0);
    else Lcd_write_string(" ");
  Lcd_write_string("Set Value"); 
}

void DisplaySetting1(uint8_t list, float uplevel, float downlevel)
{
  int temp;
  float a=uplevel;
  Lcd_gotoxy(0,0);
  if(list==0) Lcd_write_custom_char(0,0,0);
    else Lcd_write_string(" ");
  Lcd_write_string("U Level: ");
  Lcd_write_int((int)uplevel);
  uplevel = uplevel- (int)uplevel;
  temp = 10*uplevel;
  Lcd_write_string(".");
  Lcd_write_int(temp);
  if(a<10) 
  {
    Lcd_gotoxy(13,0);
    Lcd_write_string("  ");
  }
  a=downlevel;
  Lcd_gotoxy(0,1);
  if(list==1) Lcd_write_custom_char(0,1,0);
    else Lcd_write_string(" ");
  Lcd_write_string("D Level: "); 
  Lcd_write_int((int)downlevel);
  downlevel = downlevel- (int)downlevel;
  temp = 10*downlevel;
  Lcd_write_string(".");
  Lcd_write_int(temp);
  if(a<10) 
  {
    Lcd_gotoxy(13,1);
    Lcd_write_string("  ");
  }
}
void Setting()
{
  if(CheckSW())
  {
    bip();
    Lcd_clear_display();
    volatile short list=0;
    TIM3 ->CNT=0;
    uint32_t tickstart = HAL_GetTick();
    while(1)
    { 
      cnt = __HAL_TIM_GET_COUNTER(&htim3);
      //Check timeout 5s
      if((HAL_GetTick() - tickstart) > Timeout+(uint32_t)(uwTickFreq))
        goto Exit;
        
      if(__HAL_TIM_GET_COUNTER(&htim3)%4==0)
      {
        list = __HAL_TIM_GET_COUNTER(&htim3)/4;
      }
      DisplaySetting(list);   
      if(cnt>8) TIM3 ->CNT=0;
      if(cnt<0) TIM3 ->CNT=8;
      
      if(CheckSW()) 
      {
        bip();
        break;
      }
      
    }
    //Calib Mode
    if(list==0)
    {
      Lcd_clear_display();
      Lcd_gotoxy(1,0);
      Lcd_write_string("Calibrating...");
      tickstart = HAL_GetTick();
      while(1)
      {
        if((HAL_GetTick() - tickstart) > Timeout+(uint32_t)(uwTickFreq))
            goto Exit;
        if(CheckSW())
        { 
          bip();
          Lcd_gotoxy(1,0);
          Lcd_write_string("  Calibrated!   ");
          TIM2 ->CNT=0;
          HAL_Delay(1000);
          list=0xff;
          Lcd_clear_display();
          break;
        }        
      }
    }
    
    //Exit
    if(list==2)
    {
      Exit:
      Lcd_clear_display();
      WriteFlash();
      list = 0xff;    
    }
    //Set value Mode
    if(list==1)
    {
      Lcd_clear_display();
      a:
      list=0;
      TIM3 ->CNT=0;
      tickstart = HAL_GetTick();
      while(1)
      {   
        cnt = __HAL_TIM_GET_COUNTER(&htim3);
      //Check timeout 5s
        if((HAL_GetTick() - tickstart) > Timeout+(uint32_t)(uwTickFreq))
          goto Exit;
          
        if(__HAL_TIM_GET_COUNTER(&htim3)%4==0)
        {
          list = __HAL_TIM_GET_COUNTER(&htim3)/4;
        }
        DisplaySetting1(list, uplevel, downlevel);   
        if(cnt>4) TIM3 ->CNT=0;
        if(cnt<0) TIM3 ->CNT=4;
        if(!HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin))
        {            
          for( uint16_t i=0;i<100;i++)
          {
            HAL_Delay(10);
            if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)) 
            {
              bip();
              goto b;
            }
          }
          Lcd_clear_display();
          if(CheckSW())
          {
            list = 0xff;
            WriteFlash();
            bip();
            break; 
          }   
        }   
      }
      b:
      if(list==0)
      {
        TIM3 ->CNT=uplevel*8;
        tickstart = HAL_GetTick();
        while(1)
        {
          if((HAL_GetTick() - tickstart) > Timeout+(uint32_t)(uwTickFreq))
            goto Exit;
          if(uplevel<=maxlevel && downlevel <=maxlevel ) DisplaySetting1(list, uplevel, downlevel);  
          
          if(__HAL_TIM_GET_COUNTER(&htim3)%4==0)
            uplevel  =(float)__HAL_TIM_GET_COUNTER(&htim3)/8;
          
          if(uplevel>maxlevel) TIM3 ->CNT=0;
          if(uplevel<downlevel) TIM3 ->CNT=maxlevel*8;
          if(CheckSW())
          {
            bip();
            list = 0xff;
            goto a;
          }
        }
      }
      if(list==1)
      {
        TIM3 ->CNT=downlevel*8;
        tickstart = HAL_GetTick();
        while(1)
        {
          if((HAL_GetTick() - tickstart) > Timeout+(uint32_t)(uwTickFreq))
            goto Exit;
          if(downlevel<=maxlevel && downlevel <=uplevel ) DisplaySetting1(list, uplevel, downlevel);
          if(__HAL_TIM_GET_COUNTER(&htim3)%4==0)
            downlevel  =(float)__HAL_TIM_GET_COUNTER(&htim3)/8;
     
          if(downlevel>uplevel) TIM3 ->CNT=uplevel*8;
          if(CheckSW())
          {
            bip();
            list = 0xff;
            goto a;
          }
        }
      }
      
    }
    
  }
}

void CalculateLevel()
{ 
  count= __HAL_TIM_GET_COUNTER(&htim2);
  //if(__HAL_TIM_GET_COUNTER(&htim3)%4==0)
  level  = (float)__HAL_TIM_GET_COUNTER(&htim2)/13360*PI*16.3;
  if(count<0) TIM2 ->CNT=0; 
  else if(level>maxlevel) TIM2 ->CNT=maxlevel*13360/16.3/PI;
  if(level<=downlevel) Pump_status = 1;
  if(level>=uplevel) Pump_status = 0;
}
void PumpCtr()
{
  if(Pump_status) HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_RESET);
}
void AutoSave()
{
  //Auto save data every 10s
  static uint32_t preTimer=0;
  //preTimer= HAL_GetTick();
  if((HAL_GetTick() - preTimer) > 10000+(uint32_t)(uwTickFreq)) 
  {
    preTimer = HAL_GetTick();
    if(prelevel!= level || prePump_status!= Pump_status)   
    {
      prelevel= level;
      prePump_status = Pump_status;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      WriteFlash();    
    }     
    else 
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
}
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1| TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
  
  Lcd_Init();
  Lcd_create_custom_char(0, arrow_char);
  Start();
  //WriteFlash();
  ReadFlash();
  TIM2 ->CNT=count;
  //HAL_GPIO_WritePin(Pump_GPIO_Port, Pump_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
  bip();
  while (1)
  {   
    if(level<=maxlevel) DisplayPara(level,Pump_status);
    AutoSave();
    Setting();
    CalculateLevel();
    PumpCtr();
  } 
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, d4_Pin|d5_Pin|d6_Pin|d7_Pin|Pump_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : Pump_Pin */
  GPIO_InitStruct.Pin = Pump_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Pump_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : Buzz_Pin */
  GPIO_InitStruct.Pin = Buzz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzz_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : d4_Pin d5_Pin d6_Pin d7_Pin */
  GPIO_InitStruct.Pin = d4_Pin|d5_Pin|d6_Pin|d7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
