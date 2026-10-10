/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Parameter E32 - HARUS SAMA PERSIS di sender dan receiver */
#define E32_ADDH    0x00
#define E32_ADDL    0x00
#define E32_SPED    0x1A   /* UART 8N1, 9600 bps, air rate 2.4 kbps */
#define E32_CHAN    0x17   /* 410 + 0x17 = 433 MHz */
#define E32_OPTION  0x44   /* transparan, push-pull, FEC on, daya 30 dBm (1 W) */

/* Nilai e32_status (lihat di Live Expressions) */
#define E32_ST_BELUM        0
#define E32_ST_OK           1
#define E32_ST_AUX_TIMEOUT  2   /* AUX tidak pernah HIGH: cek kabel AUX / power */
#define E32_ST_NO_REPLY     3   /* tidak ada balasan: cek TX/RX tertukar, M0/M1 */
#define E32_ST_WRITE_FAIL   4   /* parameter gagal ditulis */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t counter = 0;
char msg[40];
uint8_t e32_cfg[6];               // hasil baca parameter E32 (harus C0 00 00 1A 17 47)
volatile uint8_t e32_status = E32_ST_BELUM;
volatile uint32_t tx_ok = 0;      // jumlah PING yang dikirim ke E32
volatile uint32_t aux_busy = 0;   // jumlah kali AUX terlihat LOW (modul memancar)
volatile uint32_t aux_low_ms = 0; // lama AUX LOW pada kiriman terakhir (= waktu memancar)
uint8_t e32_ver[4];               // versi modul (C3 32 xx yy, 0x32 = 433 MHz)
uint8_t usb_tx[128];              // buffer khusus kirim USB
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Tunggu AUX HIGH (modul siap). Return 1 jika siap, 0 jika timeout. */
static uint8_t E32_WaitAux(uint32_t timeout)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
  {
    if (HAL_GetTick() - t0 > timeout) return 0;
  }
  return 1;
}

/* Buang byte sisa & flag overrun di USART2, supaya balasan E32 tidak bergeser */
static void E32_FlushRx(void)
{
  for (int i = 0; i < 64; i++)
  {
    if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) && !__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) break;
    (void)huart2.Instance->DR;   /* baca SR lalu DR = clear RXNE & ORE */
  }
}

/* Ganti mode: (0,0)=normal, (1,1)=sleep/konfigurasi */
static void E32_SetMode(uint8_t m0, uint8_t m1)
{
  E32_WaitAux(1000);   /* jangan ganti mode saat modul masih sibuk */
  HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(10);
  E32_WaitAux(1000);
  HAL_Delay(5);        /* datasheet: tunggu >= 2 ms setelah AUX HIGH */
  E32_FlushRx();
}

/* Kirim perintah konfigurasi, terima 6 byte balasan (diawali 0xC0) */
static uint8_t E32_Command(const uint8_t *cmd, uint16_t len, uint8_t *resp)
{
  /* E32 menolak perintah (balas FF FF FF) jika dikirim terlalu cepat setelah perintah sebelumnya */
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)cmd, len, 100);
  if (HAL_UART_Receive(&huart2, resp, 6, 1000) != HAL_OK) return 0;
  return resp[0] == 0xC0;
}

/* Baca parameter E32; jika beda dari yang diinginkan, tulis (disimpan permanen) */
static void E32_Init(void)
{
  const uint8_t want[6] = {0xC0, E32_ADDH, E32_ADDL, E32_SPED, E32_CHAN, E32_OPTION};
  const uint8_t rd[3] = {0xC1, 0xC1, 0xC1};
  uint8_t ok = 0;

  if (!E32_WaitAux(2000))
  {
    e32_status = E32_ST_AUX_TIMEOUT;
    return;
  }

  E32_SetMode(1, 1);
  for (int i = 0; i < 3 && !ok; i++) ok = E32_Command(rd, 3, e32_cfg);

  if (!ok)
  {
    e32_status = E32_ST_NO_REPLY;
  }
  else if (memcmp(e32_cfg, want, 6) != 0)
  {
    E32_Command(want, 6, e32_cfg);
    E32_WaitAux(1000);
    HAL_Delay(50);
    ok = E32_Command(rd, 3, e32_cfg);
    e32_status = (ok && memcmp(e32_cfg, want, 6) == 0) ? E32_ST_OK : E32_ST_WRITE_FAIL;
  }
  else
  {
    e32_status = E32_ST_OK;
  }

  /* Baca versi modul */
  const uint8_t ver[3] = {0xC3, 0xC3, 0xC3};
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)ver, 3, 100);
  for (int i = 0; i < 4; i++)
  {
    if (HAL_UART_Receive(&huart2, &e32_ver[i], 1, 300) != HAL_OK) break;
  }

  E32_SetMode(0, 0);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  E32_Init();

  // Konfigurasi gagal -> LED kedip cepat 2 detik sebagai tanda
  if (e32_status != E32_ST_OK)
  {
    for (int i = 0; i < 20; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	  // Kirim hanya jika modul siap (AUX HIGH)
	  if (E32_WaitAux(500)) {
	      int len = snprintf(msg, sizeof(msg), "PING %lu\r\n", counter++);
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
	      HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);   // salinan debug ke PA9
	      tx_ok++;

	      // Pantau AUX selama 200 ms: kalau sempat LOW, modul sedang memancar
	      uint32_t t0 = HAL_GetTick();
	      while (HAL_GetTick() - t0 < 200) {
	          if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET) {
	              aux_busy++;
	              // Ukur berapa lama AUX LOW (modul sibuk memancar)
	              uint32_t t1 = HAL_GetTick();
	              E32_WaitAux(3000);
	              aux_low_ms = HAL_GetTick() - t1;
	              break;
	          }
	      }
	  }

	  // Laporan ke laptop lewat USB-C (status ikut tiap baris, karena terminal bisa dibuka belakangan)
	  int n = snprintf((char *)usb_tx, sizeof(usb_tx),
	          "[TX] %.*s | status=%u cfg=%02X %02X %02X %02X %02X %02X ver=%02X %02X tx=%lu aux=%lu aux_ms=%lu\r\n",
	          (int)strcspn(msg, "\r\n"), msg, e32_status,
	          e32_cfg[0], e32_cfg[1], e32_cfg[2], e32_cfg[3], e32_cfg[4], e32_cfg[5],
	          e32_ver[0], e32_ver[1], tx_ok, aux_busy, aux_low_ms);
	  CDC_Transmit_FS(usb_tx, n);

	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E32_M0_Pin|E32_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E32_M0_Pin E32_M1_Pin */
  GPIO_InitStruct.Pin = E32_M0_Pin|E32_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E32_AUX_Pin */
  GPIO_InitStruct.Pin = E32_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E32_AUX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
