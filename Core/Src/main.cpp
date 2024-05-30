/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "bass_drum.h"
#include "hihat.h"
#include "fm_hit.h"
#include "fx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE    128
#define MIDI_CLOCK     0xF8
#define MIDI_START     0xFA
#define MIDI_CONTINUE  0xFB
#define MIDI_STOP      0xFC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int16_t dacData[BUFFER_SIZE];
static volatile int16_t *outBufPtr = &dacData[0];
uint8_t dataReadyFlag;
const uint16_t sample_rate = 48000;

// Sinewaves
uint32_t sampleNumber = 0;
float mySinVal, sample_dt;
uint16_t sample_N;

random_device rd{};
minstd_rand gen{rd()};

Out bass_drum_out;
Out hi_hat_out;
Out fm_out;
BassDrum bass_drum(sample_rate, gen);
HiHat hi_hat(sample_rate, gen);
FmHit fm(sample_rate, gen);
FX fx(sample_rate, gen);

// USER INTERFACE;
uint8_t pot_seq_1 = 2; // pot_data[6]
uint8_t pot_seq_2 = 25; // pot_data[5]
uint8_t pot_seq_3 = 25; // pot_data[7]
uint8_t pot_seq_rd = 50; // pot_data[4]
uint8_t pot_seq_art = 50;// pot_data[3]
uint8_t pot_seq_turing = 50; // pot_data[2]
uint8_t pot_snd_1 = 25; // pot_data[10]
uint8_t pot_snd_2 = 50 - pot_map(700,50);
uint8_t pot_snd_bd = 50; // pot_data[13]
uint8_t pot_snd_hh = 50; // pot_data[9]
uint8_t pot_snd_fm = 50; // pot_data[8]
uint8_t pot_xtra = 0; // pot_data[12]
uint8_t pot_bpm = 120; // pot_data[1]
uint8_t pot_volume = 100; // pot_data[0]
// DON'T FORGET 2X LED AND CLOCK IN AND MIDI IN AND STEREO OUT
bool start_button_state = true;
bool mode_select_button_state = true; // just a placeholder so I don't forget to create it

// Sequencer
uint8_t bpm = 120;
uint8_t step = 0;
uint16_t step_sample = 0;
uint8_t stop_step = 0;
uint16_t stop_sample = 0;
bool run = false;

// Init stutter

uint16_t stutter_sample = 1;
bool stutter_bool = false;
uint8_t stutters_left = 0;
uint16_t pot_data[14];

// Initialize sequencer
bool hits[3] = { 0, 0, 0};
bool accent[3] = { 0, 0, 0};
bool stutter[3] = { 0, 0, 0};
int16_t seq_buffer[3][16] = {0};
const uint8_t steps = 16; // 8, 16 or 32
uint32_t bar_sample = (60 * sample_rate * 4) / (bpm);
uint16_t steps_sample = bar_sample / steps;
uint32_t stutter_samples[2] = { (bar_sample / 16), (bar_sample / 32) };

// Initialize MIDI
uint8_t rxByte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void ProcessMidiByte() {
    switch (rxByte) {
        case MIDI_CLOCK:
            // Handle MIDI Clock
            break;
        case MIDI_START:
        	if (run == false) {
        		step = 0;
        		step_sample = 0;
        		run = true;
        	}
            break;
        case MIDI_CONTINUE:
        	if (run == false){
        		step = stop_step;
        		step_sample = stop_sample;
        		run = true;
        	}
            break;
        case MIDI_STOP:
        	if (run == true){
        		stop_step = step;
        		stop_sample = step_sample;
        		run = false;
        	}
            break;
        default:
            break; // Ignore other messages
    }
//    midiReadyFlag = 0;
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
//    	midiReadyFlag = 1;
    	ProcessMidiByte();
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	outBufPtr = &dacData[0];
	dataReadyFlag = 1;

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	outBufPtr = &dacData[BUFFER_SIZE / 2];
	dataReadyFlag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ //interrupt handler
	if(GPIO_Pin == START_STOP_BTN_Pin && start_button_state == true){
		HAL_TIM_Base_Start_IT(&htim3);
		start_button_state = false;
	}
	else{
		__NOP();
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {
		run ^= true;
		step = 0;
		step_sample = 0;
		start_button_state = true;
		HAL_TIM_Base_Stop_IT(&htim3);
	}
}

void processData(bool run){
	for (uint8_t n = 0; n < (BUFFER_SIZE / 2) - 1; n += 2 ){
        if (step_sample % stutter_sample == 0 && stutter_bool == true) {
            hits[0] = stutter[0];
            hits[1] = stutter[1];
            hits[2] = stutter[2];
            stutters_left--;
            if (stutters_left == 0) {
                stutter_bool = false;
            }
        }
        if (step_sample == steps_sample){
            if (pot_seq_turing < 20 || pot_seq_turing > 80 ) {
                for (int i = 0; i < 3; ++i) {
                    hits[i] = seq_buffer[i][step];
                }
            } else if (stutter_bool == false) {
                if (rhythms[pot_seq_1][step] == true){
                	drum_hit(pot_seq_2,pot_seq_3,step, hits, accent);
                }
                else {
                	chance_drum_hit(pot_seq_2, pot_seq_3, pot_seq_rd, step, hits, accent);
                }
                artifacts_hit(pot_seq_2, pot_seq_rd, pot_seq_art, step, hits, accent);

                // Save hits for "turing machine"
                for (int i = 0; i < 3; ++i) {
                    seq_buffer[i][step] = hits[i];
                }
            }

            // Stutter & LED
            if ((step + 1) % 4 == 1 && run == true) {
            	HAL_GPIO_WritePin(MODE_SELECT_LED_GPIO_Port, MODE_SELECT_LED_Pin, GPIO_PIN_SET);

                // // pot_xtra defines probability of stutter between 0 and 0.1 based on pot_xtra
                stutter_bool = (rand() % 100) < (pot_xtra / 7);

                if (stutter_bool){
                    stutters_left = (rand() % 4) + 1; // How many stutters in next cycle
                    uint16_t index = (rand() % 2); // either 16 or 32th stutters
                    stutter_sample = stutter_samples[index];
                    for (int j = 0; j < 3; ++j) {
                        stutter[j] = hits[j]; // Save current hit for the stutter
                    }
                }
            } else {
            	HAL_GPIO_WritePin(MODE_SELECT_LED_GPIO_Port, MODE_SELECT_LED_Pin, GPIO_PIN_RESET);
            }

            step_sample = 0;
            ++step;
            if (step > 15) {
                step = 0;
            }
            if ((rand() % 100) < pot_xtra ) {
                fx.set_start(steps_sample);
            }
        }
        ++step_sample;

		// Generate waveform sample
		if (hits[0] == 1) {
			fm.set_start(pot_snd_1, pot_snd_2, pot_snd_fm, accent[0]);
		}
		if (hits[1] == 1) {
			bass_drum.set_start(pot_snd_1, pot_snd_2, pot_snd_bd, accent[1]);
		}
		if (hits[2] == 1) {
			hi_hat.set_start(pot_snd_1, pot_snd_2, pot_snd_hh, accent[2]);
		}

		int16_t out_l = 0;
		int16_t out_r = 0;
		if (run){
	        bass_drum_out = bass_drum.Process();
	        hi_hat_out = hi_hat.Process();
	        fm_out = fm.Process();
	        out_l = ((bass_drum_out.out_l * 10 + hi_hat_out.out_l * 20 + fm_out.out_l * 8 ) / 30);
	        out_r = ((bass_drum_out.out_r * 10 + hi_hat_out.out_r * 20 + fm_out.out_r * 8 ) / 30);
	        fx.Process(&outBufPtr[n], &outBufPtr[n + 1], &out_l, &out_r, pot_volume, 5);
		} else {
			outBufPtr[n] = (out_l);
			outBufPtr[n + 1] = (out_r);
		}

        for (int i = 0; i < 3; ++i) {
            hits[i] = 0; // Access each element using array subscript notation
        }


	}
	dataReadyFlag = 0;
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
  MX_I2S3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // UART
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

	// DMA stream for audio
	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *) dacData, BUFFER_SIZE);

	// DMA stream for ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) pot_data, 14);
	HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */



	/* USER CODE BEGIN 3 */
	if (dataReadyFlag == 1) {
		// Polling
		pot_volume = ((4096 - pot_data[0]) << 7) >> 12;
		pot_bpm = 40 + (((4096 - pot_data[1]) * 160) >> 12);
		pot_seq_turing = ((4096 - pot_data[2]) * 100) >> 12;
		pot_seq_art = ((4096 - pot_data[3]) * 100) >> 12;
		pot_seq_rd = ((4096 - pot_data[4]) * 100) >> 12;
		pot_seq_2 = ((4096 - pot_data[5]) * 50) >> 12;
		pot_seq_1 = ((4096 - pot_data[6]) * 5) >> 12;
		pot_seq_3 = ((4096 - pot_data[7]) * 50) >> 12;
		pot_snd_fm = ((4096 - pot_data[8]) * 100) >> 12;
		pot_snd_hh = ((4096 - pot_data[9]) * 100) >> 12;
		pot_snd_1 = ((4096 - pot_data[10]) * 50) >> 12;
		pot_snd_2 = ((50 - (4096 - pot_data[11])) * 50) >> 12;
		pot_xtra = ((4096 - pot_data[12]) * 100) >> 12;
		pot_snd_bd = ((4096 - pot_data[13]) * 100) >> 12;

		// Adjust BPM
		bar_sample = (60 * sample_rate * 4) / (pot_bpm);
		steps_sample = bar_sample / steps;
		stutter_samples[0] = steps_sample;
		stutter_samples[1] = (bar_sample / 32);

		// Run program
		processData(run);
	}
//	if (midiReadyFlag == 1) {
//        ProcessMidiByte();
//	}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 14;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 33600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MODE_SELECT_LED_GPIO_Port, MODE_SELECT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CLOCK_IN_Pin */
  GPIO_InitStruct.Pin = CLOCK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLOCK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_STOP_BTN_Pin MODE_SELECT_BTN_Pin */
  GPIO_InitStruct.Pin = START_STOP_BTN_Pin|MODE_SELECT_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_SELECT_LED_Pin */
  GPIO_InitStruct.Pin = MODE_SELECT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODE_SELECT_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
