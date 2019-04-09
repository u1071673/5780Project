# 5780Project
| 5780 Embedded Systems Project |
|:----|
| Marko Ljubicic |
| Nate Wilkinson |
| Seth Kingston |
| John Young |


# Pin Assignments
|Pin| Description |
|:---|:---|
|PC4|   USART3_TX (RXD) |
|PC5|   USART3_RX (TXD) |
|PC8|   Orange LED |
|PC9|   Green LED |
|PA0|   Button |
|PB5|   MOTOR Y - PWM output (IN1) |
|PB6|   MOTOR Y - PWM output (IN2) |
|PA4|   MOTOR Y - PWM TIMER 14 CH1 (ENABLE) |
|PC7|   MOTOR Y - Encoder Input TIMER 3 CH2 (ENC A) |
|PC6|   MOTOR Y - Encoder Input TIMER 3 CH1 (ENC B) |
|PB3|   MOTOR X - PWM output (IN1) |
|PB4|   MOTOR X - PWM output (IN2) |
|PB9|   MOTOR X - PWM TIMER 17 CH1 (ENABLE) |
|PA9|   MOTOR X - Encoder Input TIMER 1 CH2 (ENC A) |
|PA8|   MOTOR X - Encoder Input TIMER 1 CH1 (ENC B) |

AVAILABLE TIMERS: TIM2, TIM15, TIM16, TIM17

## VERIFIED GPIO PINS (FOR GENERAL PURPOSE OUTPUT MODE)

| WORKING PIN (ALSO USED FOR) |
|:----|
| PA0 (Button)|
| PA1 |
| PA4 (PWM TIMER 14 CH1) |
| PA5 |
| PA8 (Encoder Input TIMER 1 CH1) |
| PA9 (Encoder Input TIMER 1 CH2) |
| PA10 |
| PA15 |
| PB2 |
| PB3  |
| PB4  |
| PB5  |
| PB6  |
| PB7  |
| PB8  | 
| PB9  (PWM TIMER 17 CH1) | 
| PB10 |
| PB11 (I2C2_SDA)|
| PB12 |
| PB13 (I2C2_SCL) |
| PB14 | 
| PB15 |
| PC0 |
| PC3 |
| PC4 (USART3_TX) |
| PC5 (USART3_RX) |
| PC6 (Encoder Input TIMER 3 CH1) |
| PC7 (Encoder Input TIMER 3 CH2) |
| PC8 (Orange LED) |
| PC9 (Green LED) |
| PC10 |
| PC12 |
| PC13 |
| PC14 |
| PC15 |
| PF0 |
| PF1 |
| PD2 |

## STAY AWAY FROM THESE PINS! THEY'VE BEEN VERIFIED TO NOT WORK.
| NOT WORKING|
|:----|
| PA2 |
| PA3 |
| PA6 |
| PA7 |
| PA11|
| PA12|
| PA13|
| PA14|
| PB0 |
| PB1 |
| PC1 |
| PC2 |
