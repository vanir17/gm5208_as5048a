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
#include "as5047.h"
#include "foc.h"
#include "as5048a.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _2PI 6.28318530718f
#define CALIB_SAMPLES 2048

#define SMA_WINDOW_SIZE 5
typedef struct {
    float buffer[SMA_WINDOW_SIZE];
    float sum;
    int index;
    int count;
    float last_raw;
    float unwrapped_angle;
} AngleFilter_SMA;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t g_drv_status = 0xFF;
volatile uint16_t g_enc_diag = 0x0000;
volatile uint16_t g_enc_raw = 0;
volatile float g_mech_angle_deg = 0.0f;

volatile uint16_t g_offset_ib = 0;
volatile uint16_t g_offset_ic = 0;
uint8_t g_calib_success = 0;
volatile float angle_deg = 0.0f;


AngleFilter_SMA angle_filter;

void AngleFilter_Init(AngleFilter_SMA *f, float initial_rad) {
    f->sum = initial_rad * SMA_WINDOW_SIZE;
    for (int i = 0; i < SMA_WINDOW_SIZE; i++) {
        f->buffer[i] = initial_rad;
    }
    f->index = 0;
    f->count = SMA_WINDOW_SIZE;
    f->last_raw = initial_rad;
    f->unwrapped_angle = initial_rad;
}

// Cập nhật góc theo Radian
float AngleFilter_Update(AngleFilter_SMA *f, float raw_rad) {
    // 1. Unwrap góc tránh lỗi khi nhảy qua biên 0 <-> 2*PI
    float delta = raw_rad - f->last_raw;
    if (delta > (float)M_PI) {
        delta -= _2PI;
    } else if (delta < -(float)M_PI) {
        delta += _2PI;
    }
    
    f->unwrapped_angle += delta;
    f->last_raw = raw_rad;

    // 2. Cập nhật cửa sổ trượt O(1)
    f->sum -= f->buffer[f->index];
    f->buffer[f->index] = f->unwrapped_angle;
    f->sum += f->unwrapped_angle;

    f->index = (f->index + 1) % SMA_WINDOW_SIZE;

    // 3. Tính trung bình và chuẩn hóa về dải [0, 2*PI)
    float filtered_continuous = f->sum / (float)SMA_WINDOW_SIZE;
    float filtered_rad = fmodf(filtered_continuous, _2PI);
    if (filtered_rad < 0.0f) {
        filtered_rad += _2PI;
    }

    return filtered_rad;
}

volatile float angle_filtered_rad = 0.0f;
volatile float angle_filtered_deg = 0.0f;
volatile uint16_t raw_filtered = 0;
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
void AngleFilter_Init(AngleFilter_SMA *f, float initial_angle) {
    f->sum = initial_angle * SMA_WINDOW_SIZE;
    for (int i = 0; i < SMA_WINDOW_SIZE; i++) {
        f->buffer[i] = initial_angle;
    }
    f->index = 0;
    f->count = SMA_WINDOW_SIZE;
    f->last_raw = initial_angle;
    f->unwrapped_angle = initial_angle;
}

float AngleFilter_Update(AngleFilter_SMA *f, float raw_angle) {
    // 1. Tính delta và unwrap để theo dõi góc liên tục
    float delta = raw_angle - f->last_raw;
    if (delta > 180.0f)  delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    
    f->unwrapped_angle += delta;
    f->last_raw = raw_angle;

    // 2. Cập nhật cửa sổ trượt O(1)
    f->sum -= f->buffer[f->index];
    f->buffer[f->index] = f->unwrapped_angle;
    f->sum += f->unwrapped_angle;

    f->index = (f->index + 1) % SMA_WINDOW_SIZE;

    // 3. Trả về góc đã lọc (đưa lại về dải [0, 360))
    float filtered_continuous = f->sum / SMA_WINDOW_SIZE;
    
    // Đưa về dải 0 - 360 nếu cần góc tuyệt đối trong 1 vòng
    float filtered_angle = fmodf(filtered_continuous, 360.0f);
    if (filtered_angle < 0.0f) filtered_angle += 360.0f;

    return filtered_angle;
}
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
  AS5048A_Init(&hspi3);

  uint16_t raw_init = 0xFFFF;
  while (raw_init == 0xFFFF) 
  {
      raw_init = AS5048A_ReadRaw();
      HAL_Delay(1);
  }

  float initial_rad = ((float) raw_init / 16384.0f) * _2PI;
  AngleFilter_Init(&angle_filter, initial_rad);



  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while(1)
  {
    uint16_t raw_current = AS5048A_ReadRaw();

      // 2. Chỉ lọc khi frame SPI hợp lệ (không dính cờ lỗi)
      if (raw_current != 0xFFFF) 
      {
          g_enc_raw = raw_current;

          // Chuyển raw sang radian
          float raw_rad = ((float)raw_current / 16384.0f) * _2PI;

          // 3. Đưa qua SMA để lọc
          angle_filtered_rad = AngleFilter_Update(&angle_filter, raw_rad);

          // 4. Suy ra các định dạng khác từ giá trị đã lọc
          angle_filtered_deg = (angle_filtered_rad / _2PI) * 360.0f;
          raw_filtered = (uint16_t)((angle_filtered_rad / _2PI) * 16384.0f);
      }

      HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
