/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include <stdlib.h>

/* Custom Macros */
#define ODR_RED 6
#define ODR_BLUE 7
#define ODR_ORANGE 8
#define ODR_GREEN 9
#define AUTONOMOUS '0'
#define CONTROLLER '1'
#define OPERATION '2'
#define LED_MODE '3'
#define UP 'w'
#define DOWN 's'
#define RIGHT 'd'
#define LEFT 'a'
#define DEL 0x7F
#define BAUD_RATE 9600

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile static char received_char = 0;
volatile static uint8_t USART_new_data = 0;

void transmit_char(char c) {
	// Waiting on the USART status flag to indicate the transmit register is empty.
	//GPIOC->ODR ^= (1<<9) | (1<<8); // DEBUGGER
	while(!(USART3->ISR & USART_ISR_TXE));
	USART3->TDR = c;
}

void transmit_string(char *string) {
	int i = 0;
	while(string[i]) {
		transmit_char(string[i]);
		i++;
	}
}

/* Receives string from UART transmission and returns when user presses [enter]
   WARNING!!! The string returned must be free'd by caller*/
char *receive_string() {
	int i = 0;
	char *received_string = malloc(100);
	received_char = 0;
	USART_new_data = 0;
	while(received_char != '\n' && received_char != '\r') {
		while(!USART_new_data); // Wait for new data.
		USART_new_data = 0;
		if (received_char == DEL && i != 0) { // delete character
			transmit_char(received_char);
			received_string[i--] = 0;
		} else if ((received_char >= '0' && received_char <= 'z') || received_char == ' ') { // only transmit chars that matter.
			transmit_char(received_char);
			received_string[i++] = received_char;
		} else if (received_char != '\n' || received_char != '\r') {
			break;
		} else {
			transmit_string("\n\rInvalid Character! ");
			transmit_char(received_char);
			return NULL;
		}
	}
	received_string[i] = 0;
	return received_string;
}

/* 
parameters 
	c - r<RED>, b<BLUE>, g<GREEN>, o<ORANGE>
	op - 0<OFF>, 1<ON>, 2<TOGGLE>
*/
void operate_led(char c, uint16_t op){
	uint16_t bit_shift = 0;
	char message[25];
	message[0] = 0;
	switch(c) {
		case 'r':
			bit_shift = ODR_RED;
			strcat(message, "RED ");
			break;
		case 'b':
			bit_shift = ODR_BLUE;
		  strcat(message, "BLUE ");
			break;
		case 'o':
			bit_shift = ODR_ORANGE;
			strcat(message, "ORANGE ");
			break;
		case 'g':
			bit_shift = ODR_GREEN;
			strcat(message, "GREEN ");
			break;
		default:
			transmit_string("ERROR\n");
			return;
	}
	switch(op) {
		case 0: // Turn off the LED
			GPIOC->ODR &= ~(1<<bit_shift);
			strcat(message, "OFF!");
			break;
		case 1: // Turn on the LED
			GPIOC->ODR |= (1<<bit_shift);
			strcat(message, "ON!");

			break;
		case 2: // Toggle the LED
			GPIOC ->ODR ^= (1<<bit_shift);
			strcat(message, "TOGGLED!");
			break;
		default:
			transmit_string("ERROR\n\r");
			return;
	}
	transmit_string("\n\t");
	transmit_string(message);
}

void led_prompt() {
	transmit_string("\n\r");
	transmit_string("LED?\t");
	char *string = receive_string();
	char led_color;
	// check result
	if(!strcmp("red", string)) {
		led_color = 'r';
	} else if(!strcmp("blue", string)) {
		led_color = 'b';
	} else if (!strcmp("orange", string)) {
		led_color = 'o';
	} else if (!strcmp("green", string)) {
		led_color = 'g';
	} else {
		led_color = 0;
	}	
	free(string);
	if(led_color == 0) { // invalid input
		transmit_string("\nInvalid Color!");
		return;
	}
	transmit_string("\n\r");
	transmit_string("MODE?\t");
	string = receive_string();
	int led_mode;
	if(!strcmp("off", string)) {
		led_mode = 0;
	} else if(!strcmp("on", string)) {
		led_mode = 1;
	} else if (!strcmp("toggle", string)) {
		led_mode = 2;
	} else {
		led_mode = 3;
	}
	free(string);
	if (led_mode > 2) {  // invalid input
		transmit_string("\nInvalid Operation!");
		return;
	}
	operate_led(led_color, led_mode);
}

void operation_prompt() {
	transmit_string("\n\r");
	transmit_string("OPERATION?\t");
	char *string = receive_string();
	transmit_string("\n\r");
	// check result
	if(!strcmp("wag", string)) {
		transmit_string("\twagging...");
		// TODO: Implement
	} else if(!strcmp("crawl", string)) {
		transmit_string("\tcrawling...");
		// TODO: Implement
	} else if (!strcmp("curl", string)) {
		transmit_string("\tcurling...");
		// TODO: Implement
	} else {
		transmit_string("\tInvalid Operation!");
	}
	free(string);
}

void controller_prompt() {
	received_char = 0;
	USART_new_data = 0;
	transmit_string("\n\r");
	transmit_string("w=UP, s=DOWN, d=RIGHT, a=LEFT [enter]=exit");
	while(received_char != '\n' && received_char != '\r') {
		transmit_string("\n\r");
		while(!USART_new_data); // Wait for USART new data to arrive
		USART_new_data = 0;
		switch(received_char) {
			case UP:
				transmit_string("\tUP Pressed");
				break;
			case DOWN:
				transmit_string("\tDOWN Pressed");
				break;
			case RIGHT:
				transmit_string("\tRIGHT Pressed");
				break;
			case LEFT:				
				transmit_string("\tLEFT Pressed");
				break;
			case '\n':
			case '\r':
				break;
			default:
				transmit_string("\tUnrecognized Character: ");
				transmit_char(received_char);
				break;
		}
	}
}
void autonomous_prompt() {
}

char receive_char() {
	// Wait for RXNE=0 inside ISR
	while(!(USART3->ISR & USART_ISR_RXNE));
	return USART3->RDR;
}

void USART3_4_IRQHandler(void) {
	received_char = receive_char();
	USART_new_data = 1;
	return;
}

typedef struct _coors {
	char x;
	char y;
} coors;

// Blocking call to get back the coordinates of the left joystick.
// A line of the serial input from the controller looks like this: "PS4,<x_coor (0-255)>,<y_coor (0-255)>,...\r\n"
coors get_left_joystick_coors (void) {
  char c = 0;
  char str_start = 0;
  char on_x = 1;
  char on_y = 0;
  char xs[4] = {0, 0, 0, '\0'};
  char ys[4] = {0, 0, 0, '\0'};
  char xi = 0;
  char yi = 0;
  coors coordinates;
  while (1) {
	c = receive_char();
	// get "PS4" to start line
	if (str_start < 4) {
	  switch(c) {
	  case 'P':
		str_start++; 
		break;
	  case 'S':
		if (str_start == 1) {
		  str_start++;
		}
		break;
	  case '4':
		if (str_start == 2) {
		  str_start++; 
		}  
		break;
	  case ',':
		if (str_start == 3) {
		  str_start++;
		}
		break;
	  }
	} else {
	  // get x and y
	  if (on_x && (c == ',')) {
		// TODO: If atoi doesn't work, then use this commented out code instead for x and y
		/* char x_val; */
		/* if (xi == 1) */
		/*   x_val = xs[0]; */
		/* else if (xi == 2) */
		/*   x_val = xs[0] * 10 + xs[1]; */
		/* else */
		/*   x_val = xs[0] * 100 + xs[1] * 10 + xs[2]; */
		coordinates.x = (char) atoi(xs);
		xs[0] = 0;
		xs[1] = 0;
		xs[2] = 0;
		on_x = 0;
		xi = 0;
	  } else if (on_x) {
		xs[xi++] = c;
	  } else if (on_y && (c == ',')) {
		coordinates.y = (char) atoi(ys);
		ys[0] = 0;
		ys[1] = 0;
		ys[2] = 0;
		on_y = 0;
		yi = 0;
		str_start = 0;
		return coordinates;
	  } else {
		ys[yi++] = c;
	  }
	}
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	/* Initializing the LED GPIO pins for the red (PC6), blue (PC7), green (PC8) and orange (PC9) leds */
	/*  LED PINS INIT BEGIN */
	// Setting the pins to general-purpose output mode in the MODER register
	GPIOC->MODER |= ((1<<12) | (1<<14) | (1<<16) | (1<<18));
	// Configuring PC4 and PC5 to alternate function mode
	GPIOC->MODER |= ((1<<9) | (1<<11));
  // Setting the pin logic high
	GPIOC->ODR |= (1<<8);
	/* LED PINS INIT END */
	/* Configuring the RX and TX lines to alternate function mode */
	// AF1 for PC4 (USART3_TX) and PC5 (USART3_RX) // Note actual installation of cable is backword (PC4 is RX and PC6 is TX)
	GPIOC->AFR[0] |= (1<<20 | 1<<16);
	//GPIOC->AFR[0] &= ~(0xE<<20 | 0xE<<16);
	// Set the baud rate for communication to be 115200 bits/seconds
	USART3->BRR = HAL_RCC_GetHCLKFreq()/BAUD_RATE;
	// Enabling the transmitter and receiver hardware.
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
	// Enabling the receive register non empty interupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		USART_new_data = 0;
		received_char = 0;
		transmit_string("\n\rSELECT MODE");
		transmit_string("\n\r0 TAIL Autonomous Mode");
		transmit_string("\n\r1 TAIL Controller Mode");
		transmit_string("\n\r2 TAIL Operation Mode");
		transmit_string("\n\r3 LED COLOR Mode");
		transmit_string("\n\r");
		while(!USART_new_data); // Wait for USART new data to arrive
		switch(received_char) {
			case AUTONOMOUS:
				transmit_string("\tAUTONOMOUS Mode SELECTED");
				autonomous_prompt();
				break;
			case CONTROLLER:
				transmit_string("\tCONTROLLER Mode SELECTED");
				controller_prompt();
				break;
			case OPERATION:
				transmit_string("\tOPERATION Mode SELECTED");
				operation_prompt();
				break;
			case LED_MODE:
				transmit_string("\tLED COLOR Mode SELECTED");
				led_prompt();
				break;
			case UP:
				transmit_string("\tUP Pressed");
				break;
			case DOWN:
				transmit_string("\tDOWN Pressed");
				break;
			case RIGHT:
				transmit_string("\tRIGHT Pressed");
				break;
			case LEFT:				
				transmit_string("\tLEFT Pressed");
				break;
			case '\n':
			case '\r':
				break;
			default:
				transmit_string("\tUnrecognized Character: ");
				transmit_char(received_char);
				break;
		}
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
