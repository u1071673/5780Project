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
#include <stdio.h>
#include <stdlib.h>

/* Custom Macros */
#define ODR_RED 6
#define ODR_BLUE 7
#define ODR_ORANGE 8
#define ODR_GREEN 9
#define AUTONOMOUS '0'
#define DEBUG_MODE '1'
#define LED_MODE '2'
#define USER_MODE '3'
#define USER_MODE '3'
#define UP 'w'
#define DOWN 's'
#define RIGHT 'd'
#define LEFT 'a'
#define ROTATIONS 'r'
#define SPACE ' '
#define DEL 0x7F
#define BAUD_RATE 9600
#define MAX_Y_ENC_ROTATIONS 3.5f
#define MAX_X_ENC_ROTATIONS 3.0f

volatile int16_t error_integral_y = 0;    // Integrated error signal
volatile int16_t error_integral_x = 0;    // Integrated error signal

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
volatile int16_t motor_speed_y = 0;   // Measured motor speed 
volatile int16_t motor_speed_x = 0;   // Measured motor speed 
volatile uint8_t duty_cycle_y = 0;    // Output PWM duty cycle
volatile uint8_t duty_cycle_x = 0;    // Output PWM duty cycle
volatile int16_t target_rpm_y = 0;    // Desired speed target
volatile int16_t target_rpm_x = 0;    // Desired speed target
volatile int16_t error_y = 0;         // Speed error signal
volatile int16_t error_x = 0;         // Speed error signal
volatile uint8_t Kp_y = 8;            // Proportional gain
volatile uint8_t Kp_x = 8;            // Proportional gain
volatile uint8_t Ki_y = 8;            // Integral gain
volatile uint8_t Ki_x = 8;            // Integral gain
volatile int8_t dir_up = 0;
volatile int8_t dir_right = 0;
volatile float rotations_y = 0;
volatile float rotations_x = 0;
volatile int rest_counts = 0;
volatile int limitless = 0;
volatile int tail_needs_centering = 1;

typedef struct _coors {
	unsigned char x;
	unsigned char y;
} coors;

void SystemClock_Config(void);

/* STATIC GLOBAL VARIABLES */
volatile static char received_char = 0;
volatile static uint8_t USART_new_data = 0;

void activate_bt(void) {
	GPIOB->ODR |= (1<<2);
}
void deactivate_bt(void) {
	GPIOB->ODR &= ~(1<<2);
}

// Counter Clock wise
void y_dir_down(void) {
	  GPIOB->ODR |= (1<<5);
	  GPIOB->ODR &= ~(1<<6); 
		error_integral_y = 0;
		dir_up = 0;
}
// Clock wise
void y_dir_up(void) {
	  GPIOB->ODR &= ~(1<<5);
    GPIOB->ODR |= (1<<6);
		error_integral_y = 0;
		dir_up = 1;
}
// Counter Clock wise
void x_dir_left(void) {
		GPIOB->ODR |= (1<<3);
		GPIOB->ODR &= ~(1<<4);
		error_integral_x = 0;
		dir_right = 0;
}
// Clock wise
void x_dir_right(void) {
		GPIOB->ODR &= ~(1<<3);
		GPIOB->ODR |= (1<<4);
		error_integral_x = 0;
		dir_right = 1;
}

/* INIT METHODS */
void bt_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= (0x1<<4);
	GPIOB->ODR &= ~(1<<2);
}
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
void pwm_init(void) {
		/*________________________________________MOTOR Y________________________________________ */
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    //  Set up a pin for H-bridge PWM output (TIMER 14 CH1)
		// PA4 to AF4 for TIM14 CH1
		GPIOA->MODER |= (0x2<<8);// AF mode
		GPIOA->AFR[0] |= (0x4<<16);
    //  Set up a few GPIO output pins for direction control
		//PB5 & PB6 to general purpose output mode
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
		GPIOB->MODER &= ~(0xF<<10);
		GPIOB->MODER |= (0x1<<12) | (0x1<<10);
    //  Initialize one direction pin to high, the other low
		y_dir_up();
		/* Hint: These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     *       If you hook up the motor and the encoder reports you are
     *       running in reverse, either swap the direction pins or the
     *       encoder pins. (we'll only be using forward speed in this lab)
     */
	
		/*________________________________________MOTOR X________________________________________ */
    //  Set up a pin for H-bridge PWM output (TIMER 17 CH1)
		//  PB9 to AF2 for TIM17 CH1
		GPIOB->MODER &= ~(0x3<<18); // AF mode
		GPIOB->MODER |= (0x2<<18);  // AF mode
		GPIOB->AFR[1] &= ~(0xF<<4);
		GPIOB->AFR[1] |= (0x2<<4);
    //  Set up a few GPIO output pins for direction control
		// PB3 & PB4
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		GPIOB->MODER &= ~(0xF<<6);
		GPIOB->MODER |= (0x5<<6);
		//  Initialize one direction pin to high, the other low
		x_dir_right();
		/* Hint: These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     *       If you hook up the motor and the encoder reports you are
     *       running in reverse, either swap the direction pins or the
     *       encoder pins. (we'll only be using forward speed in this lab)
     */
		
		/*________________________________________MOTOR Y________________________________________ */
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
		
		/*________________________________________MOTOR X________________________________________ */
    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->CR1 = 0;                         // Clear control registers
    TIM17->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM17->CCER = 0;
    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM17->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM17->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1 depending on (CCxE, CCxNE, MOE, OSSI and OSSR)
		// OC1 signal is output on the corresponding output opin depending on 
		//MOE = 1
		TIM17->BDTR |= TIM_BDTR_MOE;						// Enable outputs if their respective enable bits are set (CCxE, CCxNE);
		//OSSI = 1
		TIM17->BDTR |= TIM_BDTR_OSSI;						// When inactive, OC/OCN outputs are forced first with their idle level as soon as CCxE=1 or CCxNE=1. The OC/OCN enable output signal=1
		//OSSR = 1
		TIM17->BDTR |= TIM_BDTR_OSSR;						// When inactive, OC/OCN outputs are enabled with their inactrive level as soon as CCxE=1 or CCxNE=1. Then enable output signal=1;
		TIM17->PSC = 1;                         // Run timer on 24Mhz
    TIM17->ARR = 1200;                      // PWM at 20kHz
    TIM17->CCR1 = 0;                        // Start PWM at 0% duty cycle
		
    TIM17->CR1 |= TIM_CR1_CEN;              // Enable timer
}
// Sets up encoder interface to read motor speed
void encoder_init(void) {
		/*________________________________________MOTOR Y________________________________________ */
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    //  Set up encoder input pins (TIMER 3 CH1 and CH2)
		// PC6 PC7
		GPIOC->MODER |= (0x2<<14);
		GPIOC->MODER &= ~(0x1<<14);
		GPIOC->MODER |= (0x2<<12);
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
		
		
		/*________________________________________MOTOR X________________________________________ */
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		//  Set up encoder input pins (TIMER 1 CH1 and CH2)
		// PA8 & PA9
		GPIOA->MODER &= ~(0xF<<16);
		GPIOA->MODER |= (0x2<<18) | (0x2<<16);
		// AF2
		GPIOA->AFR[1] &= ~(0xFF<<0);
		GPIOA->AFR[1] |= (0x2<<0);
		GPIOA->AFR[1] |= (0x2<<4);
    /* Hint: MAKE SURE THAT YOU USE 5V TOLERANT PINS FOR THE ENCODER INPUTS!
     *       You'll fry the processor otherwise, read the lab to find out why!
     */

    // Set up encoder interface (TIM1 encoder input mode)
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CCMR1 = 0;    //Clear control registers
    TIM1->CCER = 0;
    TIM1->SMCR = 0;
    TIM1->CR1 = 0;

    TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM1->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM1->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM1->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM1->CR1 |= TIM_CR1_CEN;                               // Enable timer
		
		/* MOTOR Y & X */
    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    //  Select PSC and ARR values that give an appropriate interrupt rate
		
    /* Hint: See section in lab on sampling rate!
     *       Recommend choosing a sample rate that gives 2:1 ratio between encoder value
     *       and target speed. (Example: 200 RPM = 400 Encoder count for interrupt period)
     *       This is so your system will match the lab solution
     */
    TIM6->PSC = 0x0007;
		TIM6->ARR = 0x927C; 
		
    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}
// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init();
    encoder_init();
}



/* METHODS TO STOP THE TAIL FROM MOVING INTO TOO STRESSFULL POSITIONS */
int proceed_to_rotate_y() {
	if(limitless)
		return 1;
	if(dir_up) {
		if((rotations_y < MAX_Y_ENC_ROTATIONS && rotations_x < (MAX_X_ENC_ROTATIONS/2.0f) && rotations_x > -(MAX_X_ENC_ROTATIONS/2.0f)) || rotations_y < (MAX_Y_ENC_ROTATIONS/2.0f))
			return 1;
	}	else { // y_rotating_down
		if((rotations_y > -MAX_Y_ENC_ROTATIONS && rotations_x < (MAX_X_ENC_ROTATIONS/2.0f) && rotations_x > -(MAX_X_ENC_ROTATIONS/2.0f)) || rotations_y > -(MAX_Y_ENC_ROTATIONS/2.0f))
			return 1;
	}
	return 0;
}

int proceed_to_rotate_x() {
	if(limitless)
		return 1;
	if(dir_right) {
		if((rotations_x < MAX_X_ENC_ROTATIONS && rotations_y < (MAX_Y_ENC_ROTATIONS/2.0f) && rotations_y > -(MAX_Y_ENC_ROTATIONS/2.0f)) || rotations_x < (MAX_X_ENC_ROTATIONS/2.0f))
			return 1;
	}	else { // y_rotating_down
		if((rotations_x > -MAX_X_ENC_ROTATIONS && rotations_y < (MAX_Y_ENC_ROTATIONS/2.0f) && rotations_y > -(MAX_Y_ENC_ROTATIONS/2.0f)) || rotations_x > -(MAX_X_ENC_ROTATIONS/2.0f))
			return 1;
	}
	return 0;
}

int tail_is_center_x() {
	if(rotations_x < 0.1f && rotations_x > -0.1f)
		return 1;
	return 0;
	
}
int tail_is_center_y() {
	if(rotations_y < 0.2f && rotations_y > -0.2f)
		return 1;
	return 0;
}

void center_tail_x() {
	if(tail_is_center_x()) {
		target_rpm_x = 0;
		return;
	}
	if(dir_right) {
		if(rotations_x > 0.1f)
				target_rpm_x = -60;
	} else { // dir_left
		if(rotations_x < -0.1f) {
				target_rpm_x = 60;
		}
	}
}
void center_tail_y() {
	if(tail_is_center_y()) {
		target_rpm_y = 0;
		return;
	}
	if(dir_up) {
		if(rotations_y > 0.2f)
				target_rpm_y = -60;
	} else { // dir_down
		if(rotations_y < -0.2f) {
				target_rpm_y = 60;
		}
	}
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_y(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;// Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_x(uint8_t duty) {
    if(duty <= 100) {
			 TIM17->CCR1 = ((uint32_t)duty*TIM17->ARR)/100;// Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
   }
}

void PI_update_y(void) {
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
		int16_t motor_rpm = motor_speed_y/2;

		if (motor_rpm == 0) {
			if (dir_up && target_rpm_y < 0) {
				y_dir_down();
			} else if (!dir_up && target_rpm_y > 0) {
				y_dir_up();
			}
		}
		int16_t output = 0;
		if(proceed_to_rotate_y()) {
			//  calculate error signal and write to "error" variable
			if(dir_up)
			{
				int16_t t_rpm = target_rpm_y;
				t_rpm = t_rpm < 0 ? 0 : t_rpm;
				error_y = t_rpm - motor_rpm;
						/* Hint: Remember that your calculated motor speed may not be directly in RPM!
				 *       You will need to convert the target or encoder speeds to the same units.
				 *       I recommend converting to whatever units result in larger values, gives
				 *       more resolution.
				 */

				//  Calculate integral portion of PI controller, write to "error_integral" variable
				error_integral_y = error_integral_y + (Ki_y*error_y);
				//  Clamp the value of the integral to a limited positive range
				error_integral_y = error_integral_y < 3200 ? error_integral_y : 3200;
				error_integral_y = error_integral_y < 0 ? 0 : error_integral_y;
				/* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
				 *       You'll read more about this for the post-lab. The exact value is arbitrary
				 *       but affects the PI tuning.
				 *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
				 */
			}
			else {			
				int16_t t_rpm = target_rpm_y;
				t_rpm = t_rpm > 0 ? 0 : t_rpm;
				error_y = (t_rpm ) - motor_rpm;
						/* Hint: Remember that your calculated motor speed may not be directly in RPM!
				 *       You will need to convert the target or encoder speeds to the same units.
				 *       I recommend converting to whatever units result in larger values, gives
				 *       more resolution.
				 */

				//  Calculate integral portion of PI controller, write to "error_integral" variable
				error_integral_y = error_integral_y + (Ki_y*error_y);
				//  Clamp the value of the integral to a limited negative range
				error_integral_y = error_integral_y < 0 ? error_integral_y : 0;
				error_integral_y = error_integral_y < -3200 ? -3200 : error_integral_y;
				/* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
				 *       You'll read more about this for the post-lab. The exact value is arbitrary
				 *       but affects the PI tuning.
				 *       Recommend that you clamp between -3200 and 0 (what is used in the lab solution)
				 */
			}
			output = Kp_y*error_y + error_integral_y;
		}

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

     //  Divide the output into the proper range for output adjustment
		output = abs(output) >> 5;
		
     //  Clamp the output value between 0 and 100
		output = output;
		output = output > 100 ? 100 : output;
		output = output < 0 ? 0 : output;
		
    pwm_setDutyCycle_y(output);
    duty_cycle_y = output;            // For debug viewing
}
void PI_update_x(void) {
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
		int16_t motor_rpm = motor_speed_x/2;

		if (motor_rpm == 0) {
			if (dir_right && target_rpm_x < 0) {
				x_dir_left();
			} else if (!dir_right && target_rpm_x > 0) {
				x_dir_right();
			}
		}
		int16_t output = 0;
		if(proceed_to_rotate_x()) {
			//  calculate error signal and write to "error" variable
			if(dir_right)
			{
				int16_t t_rpm = target_rpm_x;
				t_rpm = t_rpm < 0 ? 0 : t_rpm;
				error_x = t_rpm - motor_rpm;
						/* Hint: Remember that your calculated motor speed may not be directly in RPM!
				 *       You will need to convert the target or encoder speeds to the same units.
				 *       I recommend converting to whatever units result in larger values, gives
				 *       more resolution.
				 */

				//  Calculate integral portion of PI controller, write to "error_integral" variable
				error_integral_x = error_integral_x + (Ki_x*error_x);
				//  Clamp the value of the integral to a limited positive range
				error_integral_x = error_integral_x < 3200 ? error_integral_x : 3200;
				error_integral_x = error_integral_x < 0 ? 0 : error_integral_x;
				/* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
				 *       You'll read more about this for the post-lab. The exact value is arbitrary
				 *       but affects the PI tuning.
				 *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
				 */
			}
			else {			
				int16_t t_rpm = target_rpm_x;
				t_rpm = t_rpm > 0 ? 0 : t_rpm;
				error_x = (t_rpm ) - motor_rpm;
						/* Hint: Remember that your calculated motor speed may not be directly in RPM!
				 *       You will need to convert the target or encoder speeds to the same units.
				 *       I recommend converting to whatever units result in larger values, gives
				 *       more resolution.
				 */

				//  Calculate integral portion of PI controller, write to "error_integral" variable
				error_integral_x = error_integral_x + (Ki_x*error_x);
				//  Clamp the value of the integral to a limited negative range
				error_integral_x = error_integral_x < 0 ? error_integral_x : 0;
				error_integral_x = error_integral_x < -3200 ? -3200 : error_integral_x;
				/* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
				 *       You'll read more about this for the post-lab. The exact value is arbitrary
				 *       but affects the PI tuning.
				 *       Recommend that you clamp between -3200 and 0 (what is used in the lab solution)
				 */
			}
			output = Kp_x*error_x + error_integral_x;
		}

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

     //  Divide the output into the proper range for output adjustment
		output = abs(output) >> 5;
		
     //  Clamp the output value between 0 and 100
		output = output;
		output = output > 100 ? 100 : output;
		output = output < 0 ? 0 : output;
    pwm_setDutyCycle_x(output);
    duty_cycle_x = output;            // For debug viewing
}
// Encoder interrupt to calculate motor speed, also manages PI controller
// This method is called once every 37.5 ms.
void TIM6_DAC_IRQHandler(void) {

		/*________________________________________MOTOR Y________________________________________ */
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    motor_speed_y = (TIM3->CNT - 0x7FFF);
		rotations_y += ((float)motor_speed_y/3000.0f);
    TIM3->CNT = 0x7FFF; // Reset back to center point
		/*________________________________________MOTOR X________________________________________ */
		/* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
		motor_speed_x = (TIM1->CNT - 0x7FFF);
		rotations_x += ((float)motor_speed_x/3000.0f);
    TIM1->CNT = 0x7FFF; // Reset back to center point

		/*________________________________________MOTOR Y________________________________________ */
    // Call the PI update function
		GPIOC->ODR ^= (1<<ODR_ORANGE);
    PI_update_y();
		/*________________________________________MOTOR X________________________________________ */
		PI_update_x();
	
		/* MOTOR X & Y */
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
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
void debug_prompt() {
	received_char = 0;
	USART_new_data = 0;
	limitless = 1;
	target_rpm_x = 0;
	target_rpm_y = 0;
	transmit_string("\n\r");
	transmit_string("w=UP, s=DOWN, d=RIGHT, a=LEFT, r=RESET ROTATIONS, [SPACE]=stop [enter]=exit");
	while(received_char != '\n' && received_char != '\r') {
		transmit_string("\n\r");
		while(!USART_new_data); // Wait for USART new data to arrive
		USART_new_data = 0;
		switch(received_char) {
			case UP:
				transmit_string("\tGOING UP");
				target_rpm_y = 100;
				break;
			case DOWN:
				transmit_string("\tGOING DOWN");
				target_rpm_y = -100;
				break;
			case RIGHT:
				transmit_string("\tGOING RIGHT");
				target_rpm_x = 100;
				break;
			case LEFT:
				transmit_string("\tGOING LEFT");
				target_rpm_x = -100;
				break;
			case SPACE:
				transmit_string("\tSTOPPING TAIL");
				target_rpm_x = 0;
				target_rpm_y = 0;
				break;
			case ROTATIONS:
				transmit_string("\tROTATIONS RESET");
				rotations_x = 0;
				rotations_y = 0;
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
		limitless = 0;
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
		received_char = 0;
		USART_new_data = 0;
		while(!USART_new_data);
		c = received_char;
		//transmit_char(c);
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

void main_menu() {
	transmit_string("\n\r\tSELECT MODE");
	transmit_string("\n\r1 \t\tDEBUG Mode");
	transmit_string("\n\r2 \t\tLED COLOR Mode");
	transmit_string("\n\r3 \t\tUSER Mode");
	transmit_string("\n\r[ANY KEY] \tCenter Tail");
	transmit_string("\n\r");
	USART_new_data = 0;
	received_char = 0;
	tail_needs_centering = 1;
	deactivate_bt();
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
	bt_init();
	LED_init();
	USART_init();
	button_init();
	motor_init();
	coors coordinates;
	main_menu();
	while (1)
	{
		if (USART_new_data) {
			target_rpm_x = 0;
			transmit_char(received_char);
			switch(received_char) {
			case DEBUG_MODE:
				transmit_string("\tDEBUG Mode SELECTED");
				debug_prompt();
				break;
			case LED_MODE:
				transmit_string("\tLED COLOR Mode SELECTED");
				led_prompt();
				break;
			case USER_MODE:
				// Disable USART interrupts b/c there would be too many from constant char stream
				transmit_string("\tUSER Mode SELECTED\n\r");
			  activate_bt();
				rest_counts = 0;
				while (1) {
					get_left_joystick_coors(&coordinates);
					target_rpm_x = ((coordinates.x * 1176) / 1000) - 150;
					target_rpm_y = ((coordinates.y * 1176) / 1000) - 150;
					target_rpm_x = target_rpm_x < 20 && target_rpm_x > -20 ? 0 : target_rpm_x;
					target_rpm_y = target_rpm_y < 20 && target_rpm_y > -20 ? 0 : target_rpm_y;
					target_rpm_y = -target_rpm_y;
					if ((target_rpm_x == 0) && (target_rpm_y == 0))
						++rest_counts;
					else {
						GPIOC->ODR ^= (1<<ODR_GREEN);
						rest_counts = 0;
					}
					if (rest_counts >= 200) { 
						deactivate_bt();
						break;
					}
				}
			case '\n':
			case '\r':
			default:
				transmit_string("\tCENTERING...");
				break;
			}
			main_menu();
		} else {
			// We're in autonomous mode
			if(tail_needs_centering){
				center_tail_x();
				center_tail_y();
				if(tail_is_center_x() && tail_is_center_y() && motor_speed_x == 0 && motor_speed_y == 0) {
					tail_needs_centering = 0;
					target_rpm_x = 0;
					target_rpm_y = 0;
				}
			} else {
				target_rpm_y = 80;
				if(rotations_y > 3.45f) {
					if(dir_right) {
						if(proceed_to_rotate_x()){
							target_rpm_x = 80;
						} else {
							target_rpm_x = -80;
						}
					} else {
						if(proceed_to_rotate_x()){
							target_rpm_x = -80;
						} else {
							target_rpm_x = 80;
						}
					}
				}
			}
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
