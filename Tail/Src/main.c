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
#define BLUETOOTH '4'
#define UP 'w'
#define DOWN 's'
#define RIGHT 'd'
#define LEFT 'a'
#define DEL 0x7F
#define BAUD_RATE 9600
#define MAX_Y_ENC_ROTATIONS 4
#define MAX_X_ENC_ROTATIONS 4

typedef struct _coors {
	unsigned char x;
	unsigned char y;
} coors;

void SystemClock_Config(void);

/* STATIC GLOBAL VARIABLES */
volatile static char received_char = 0;
volatile static uint8_t USART_new_data = 0;
volatile static int16_t y_motor_speed = 0;   // Measured motor speed
volatile static int16_t y_target_rpm = 0;    // Desired speed target
volatile static int8_t y_adc_value = 0;      // ADC measured motor current
volatile static uint8_t y_Kp = 8;            // Proportional gain
volatile static uint8_t y_Ki = 8;            // Integral gain
volatile static int8_t x_encoder_rotations = 0;
volatile static int8_t y_encoder_rotations = 0;
volatile static uint8_t x_rotating_right = 0;
volatile static uint8_t y_rotating_up = 0;


/* INIT METHODS */
void LED_init() {
	// Enable peripheral clock to GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	/* Initializing the LED GPIO pins for the red (PC6), blue (PC7), green (PC8) and orange (PC9) leds */
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0; 
	// Setting the pins to general-purpose output mode in the MODER register (low speed)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
	GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR6_0 | GPIO_OSPEEDR_OSPEEDR6_1) |
											(GPIO_OSPEEDR_OSPEEDR7_0 | GPIO_OSPEEDR_OSPEEDR7_1) |
											(GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
											(GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   
	
	// Set to no pull-up/down
	GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1) |
										(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1) |
										(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
										(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));
	// Shut off all LED's
	GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9); 
}
void  button_init() {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}
void USART_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	// Configuring PC4 and PC5 to alternate function mode
	GPIOC->MODER |= ((1<<9) | (1<<11));	
	/* Configuring the RX and TX lines to alternate function mode */
	// AF1 for PC4 (USART3_TX) and PC5 (USART3_RX) // Note: actual installation of cable is backword (PC4 is RX and PC6 is TX)
	GPIOC->AFR[0] |= (1<<20 | 1<<16);
	GPIOC->AFR[0] &= ~(0xE<<20 | 0xE<<16);
	// Set the baud rate for communication to be 115200 bits/seconds
	USART3->BRR = HAL_RCC_GetHCLKFreq()/BAUD_RATE;
	// Enabling the transmitter and reciever hardware.
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
	// Enabling the receive register non empty interupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
}
// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init_y(void) {
		//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    /// TODO: Set up a pin for H-bridge PWM output (TIMER 14 CH1)
		// PA4 to AF4 for TIM14 CH1
		GPIOA->MODER |= (0x2<<8);// AF mode
		GPIOA->AFR[0] |= (0x4<<16);
    /// TODO: Set up a few GPIO output pins for direction control
		// PA1 & PA2 to general purpose output mode
		GPIOA->MODER |= (0x1<<4) | (0x1<<2);
    /// TODO: Initialize one direction pin to high, the other low
		// PA1 = OFF; PA2 = OFF
		GPIOA->BSRR |= (0x1<<1);
    /* Hint: These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     *       If you hook up the motor and the encoder reports you are
     *       running in reverse, either swap the direction pins or the
     *       encoder pins. (we'll only be using forward speed in this lab)
     */
    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;
		
    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle

    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}
// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_y(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;// Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}
// Sets up encoder interface to read motor speed
void encoder_init_y(void) {
    /// TODO: Set up encoder input pins (TIMER 3 CH1 and CH2)
		// PC6 PC7
		GPIOC->MODER |= (0x2<<14); // Setting PC7 to alternate function mode
		GPIOC->MODER &= ~(0x1<<14); 
		GPIOC->MODER |= (0x2<<12); // Setting PC6 to alternate function mode.
		GPIOC->MODER &= ~(0x1<<12);
	  GPIOC->AFR[0] &= ~(0xF<<24);
		GPIOC->AFR[0] &= ~(0x7<<28);
    /* Hint: MAKE SURE THAT YOU USE 5V TOLERANT PINS FOR THE ENCODER INPUTS!
     *       You'll fry the processor otherwise, read the lab to find out why!
     */

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;    //Clear control registers
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer
		

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    /// TODO: Select PSC and ARR values that give an appropriate interrupt rate
		
    /* Hint: See section in lab on sampling rate!
     *       Recommend choosing a sample rate that gives 2:1 ratio between encoder value
     *       and target speed. (Example: 200 RPM = 400 Encoder count for interrupt period)
     *       This is so your system will match the lab solution
     */
         TIM6->PSC = 0x0007; // TODO: Change this!
         TIM6->ARR = 0x927C; // TODO: Change this!

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}
void PI_update(void) {
    /* Run PI control loop
     *
     * Make sure to use the indicated variable names. This allows STMStudio to monitor
     * the condition of the system!
     *
     * target_rpm -> target motor speed in RPM
     * motor_speed -> raw motor speed in encoder counts
     * error -> error signal (difference between measured speed and target)
     * error_integral -> integrated error signal
     * Kp -> Proportional Gain
     * Ki -> Integral Gain
     * output -> raw output signal from PI controller
     * duty_cycle -> used to report the duty cycle of the system
     * adc_value -> raw ADC counts to report current
     *
     */
		int16_t y_motor_rpm = y_motor_speed/2;
    /// TODO: calculate error signal and write to "error" variable
		int16_t error = y_target_rpm - y_motor_rpm;
		
    /* Hint: Remember that your calculated motor speed may not be directly in RPM!
     *       You will need to convert the target or encoder speeds to the same units.
     *       I recommend converting to whatever units result in larger values, gives
     *       more resolution.
     */

    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
		int16_t y_error_integral = y_error_integral + (y_Ki*error);
    /// TODO: Clamp the value of the integral to a limited positive range
		y_error_integral = y_error_integral < 3200 ? y_error_integral : 3200;
		y_error_integral = y_error_integral < 0 ? 0 : y_error_integral;
    /* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
     *       You'll read more about this for the post-lab. The exact value is arbitrary
     *       but affects the PI tuning.
     *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
     */

    /// TODO: Calculate proportional portion, add integral and write to "output" variable
    int16_t y_output = y_Kp*error + y_error_integral; // Change this!

    /* Because the calculated values for the PI controller are significantly larger than
     * the allowable range for duty cycle, you'll need to divide the result down into
     * an appropriate range. (Maximum integral clamp / X = 100% duty cycle)
     *
     * Hint: If you chose 3200 for the integral clamp you should divide by 32 (right shift by 5 bits),
     *       this will give you an output of 100 at maximum integral "windup".
     *
     * This division also turns the above calculations into pseudo fixed-point. This is because
     * the lowest 5 bits act as if they were below the decimal point until the division where they
     * were truncated off to result in an integer value.
     *
     * Technically most of this is arbitrary, in a real system you would want to use a fixed-point
     * math library. The main difference that these values make is the difference in the gain values
     * required for tuning.
     */

     /// TODO: Divide the output into the proper range for output adjustment
		y_output = y_output >> 5;
		
     /// TODO: Clamp the output value between 0 and 100
		y_output = y_output > 100 ? 100 : y_output;
		y_output = y_output < 0 ? 0 : y_output;
		
    pwm_setDutyCycle_y(y_output);

    // Read the ADC value for current monitoring, actual conversion into meaningful units
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        y_adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    y_motor_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point

    // Call the PI update function
    PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void ADC_init(void) {

    /// TODO: Configure a pin for ADC input (used for current monitoring)
		GPIOA->MODER |= (0x3<<4); // PA2 bits (5:4) ANOLOGUE MODE

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;
    ADC1->CFGR1 |= (ADC_CFGR1_CONT);        // Set to continuous mode and 12-bit resolution

    /// TODO: Enable the proper channel for the ADC pin you are using
    ADC1->CHSELR |= ADC_CHSELR_CHSEL2; // Change this!

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}
void motor_init_y(void) {
    pwm_init_y();
    encoder_init_y();
    ADC_init();
}
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

/* USART PROMPT METHODS */
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

/* METHODS TO STOP THE TAIL FROM MOVING INTO TOO STRESSFULL POSITIONS */
int proceed_to_rotate_x() {
	if((x_encoder_rotations < MAX_X_ENC_ROTATIONS) && (x_encoder_rotations > -MAX_X_ENC_ROTATIONS) &&
		 (y_encoder_rotations < MAX_Y_ENC_ROTATIONS>>1) && (y_encoder_rotations > -MAX_Y_ENC_ROTATIONS>>1)) {
			 return 1;
	}
	return 0;
}
int proceed_to_rotate_y() {
	if((y_encoder_rotations < MAX_Y_ENC_ROTATIONS) && (y_encoder_rotations > -MAX_Y_ENC_ROTATIONS) &&
		 (x_encoder_rotations < MAX_X_ENC_ROTATIONS>>1) && (x_encoder_rotations > -MAX_X_ENC_ROTATIONS>>1)) {
			 return 1;
	}
	return 0;
}

// Blocking call to get back the coordinates of the left joystick.
// A line of the serial input from the controller looks like this: "PS4,<x_coor (0-255)>,<y_coor (0-255)>,...\r\n"
void get_left_joystick_coors (coors* coordinates) {
	char c;
	char str_index = 0;
	char err = 0;
	char on_x = 1;
	char xs[4] = {'\0', '\0', '\0', '\0'};
	char ys[4] = {'\0', '\0', '\0', '\0'};
	char i = 0;
	while (1) {
		if (err)
			break;
		c = receive_char();
		if ((str_index == 0) && c == 'P') {
			str_index++;
		} else if ((str_index == 1) && c == 'S') {
			str_index++;
		} else if ((str_index == 2) && c == '4') {
			str_index++;
		} else if ((str_index == 3) && c == ',') {
			str_index++;
		} else if (str_index > 3) {
			if (i > 3)
					err = 1;
			if (on_x) {
				if (c == ',') {
					coordinates->x = atoi(xs);
					on_x = 0;
					i = 0;
				} else if ((c >= '0') && (c <= '9')) {
					xs[i] = c;
					i++;
				} else {
					err = 1;
				}
			} else {
				if (c == ',') {
					coordinates->y = atoi(ys);
					break;
				} else if ((c >= '0') && (c <= '9')) {
					ys[i] = c;
					i++;
				} else {
					err = 1;
				}
			}
			str_index++;
		}
	}
	if (err)
		transmit_char('E');
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	LED_init();
	USART_init();
	button_init();
	motor_init_y();
	coors coordinates;
  while (1)
  {
		USART_new_data = 0;
		received_char = 0;
		transmit_string("\n\rSELECT MODE");
		transmit_string("\n\r0 TAIL Autonomous Mode");
		transmit_string("\n\r1 TAIL Controller Mode");
		transmit_string("\n\r2 TAIL Operation Mode");
		transmit_string("\n\r3 LED COLOR Mode");
		transmit_string("\n\r4 BLUETOOTH Mode");
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
			case BLUETOOTH:
				// Disable USART interrupts b/c there would be too many from constant char stream
				NVIC_DisableIRQ(USART3_4_IRQn);
				transmit_string("\tBLUETOOTH Mode SELECTED\n\r");
				while (1) {
					get_left_joystick_coors(&coordinates);
					if (coordinates.x <= 50)
						transmit_char('L');
					else if (coordinates.x <= 200)
						transmit_char('M');
					else
						transmit_char('R');
					if (coordinates.y <= 50)
						transmit_char('T');
					else if (coordinates.y <= 200)
						transmit_char('C');
					else
						transmit_char('B');
				}
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
