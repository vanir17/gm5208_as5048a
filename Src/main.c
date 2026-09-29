/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "drv8301.h"
#include "foc.h"
#include "as5048a.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SMA_WINDOW_SIZE 8
#define ENC_ERR_LIMIT 50
#define CALIB_VOLTAGE 0.35f

typedef struct
{
  uint16_t buf[SMA_WINDOW_SIZE];
  uint8_t idx;

}AngleSMA_t;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_drv_status = 0xFF;
volatile uint16_t g_enc_raw = 0;
volatile float    g_enc_filt_counts = 0.0f;
volatile float    g_enc_filt_deg   = 0.0f;
volatile uint16_t g_enc_err        = 0;
volatile uint8_t  g_fault          = 0;
volatile uint32_t g_isr_cycles     = 0;      /* so chu ky CPU cua 1 lan ngat (168 cycles = 1us) */

static AngleSMA_t g_sma;

volatile float current_angle = 0.0f;
volatile uint16_t current_raw = 0;

volatile float g_target_vq = 0.2f;





volatile float    dbg_theta_elec = 0.0f; /* Góc điện đưa vào Park/Clarke */
volatile float    dbg_vd = 0.0f;
volatile float    dbg_vq = 0.0f;
volatile uint16_t dbg_ccr1 = 0;          /* Duty cycle kênh A */
volatile uint16_t dbg_ccr2 = 0;          /* Duty cycle kênh B */
volatile uint16_t dbg_ccr3 = 0;          /* Duty cycle kênh C */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void AngleSMA_Init(AngleSMA_t *f, uint16_t raw)
{
  for(int i = 0; i < SMA_WINDOW_SIZE; i++)
  {
    f->buf[i] = raw;
  }

  f->idx = 0;
}

static float AngleSMA_Update(AngleSMA_t *f, uint16_t raw)
{
  f->buf[f->idx] = raw;
  f->idx = (uint8_t)((f->idx + 1) & (SMA_WINDOW_SIZE - 1));

  int32_t sum = 0;
  for(int i = 0; i < SMA_WINDOW_SIZE; i++)
  {
    int32_t d = (int32_t)f->buf[i] - (int32_t)raw;
    d = ((d + 8192) & 16383) - 8192; /* wrap [-8192 ; 8191]*/
    sum += d;
  }

  float out = (float)raw + (float)sum * (1.0f / (float)SMA_WINDOW_SIZE);

  if(out < 0.0f) out += 16384.0f;
  if(out > 16384.0f) out -= 16384.0f;

  return out;
}


static uint16_t WaitValidRaw(void)
{
  uint16_t r;
  do
  {
    r = AS5048A_ReadRaw();
    // for(volatile int i = 0; i < 5000; i++)
    // {
    //   __NOP();
    // }
    HAL_Delay(1);
  } while((r == AS5048A_ERR));

  return r;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


  //1.Encoder - AS5048A
  if(AS5048A_Init(&hspi3) != 0)
  {
    Error_Handler();
  }
  (void)WaitValidRaw();

  //2. Driver - DRV8301
  g_drv_status = DRV8301_M1_Init(DRV8301_GAIN_20VpV);
  if(g_drv_status != 0)
  {
    Error_Handler();
  }

  //3. Set duty 50% (0V) before starting
  FOC_M1_Init();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); /*EN_GATE DRV8301*/


  //Enable TIM8 for controlling Motor 1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  TIM8->BDTR |= TIM_BDTR_MOE; //Enable H bridge

  //4. Align Motor
  if(FOC_AutoCalibrate(CALIB_VOLTAGE) != 0)
  {
    Error_Handler(); //check g_calib_err
  }
  
  //5. SMA Init
  AngleSMA_Init(&g_sma, WaitValidRaw());
  AS5048A_Prime();

  //6. Enable TIM8 ISR Function
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim8);

  /* USER CODE END 2 */


  /* USER CODE BEGIN WHILE */
  
  while(1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance == TIM8)
  {
    uint32_t t0 = DWT->CYCCNT;

    uint16_t raw = AS5048A_ReadFast();
    if(raw != AS5048A_ERR)
    {
      g_enc_err = 0;
      g_enc_raw = raw;
      float ang = AngleSMA_Update(&g_sma, raw); //count
      g_enc_filt_counts = ang;
      g_enc_filt_deg = ang * (360.0f / 16384.0f);

      FOC_VoltageMode_Step(ang, g_target_vq);
      dbg_ccr1 = TIM8->CCR1;
    dbg_ccr2 = TIM8->CCR2;
    dbg_ccr3 = TIM8->CCR3;
    }
    else if( ++g_enc_err > ENC_ERR_LIMIT)
    {
      FOC_M1_EmergencyStop();
      g_fault = 1;
    }
    g_isr_cycles = DWT->CYCCNT - t0;
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  FOC_M1_EmergencyStop();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
