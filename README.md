# 5780Project
5780 Embedded Systems Project
Marko Ljubicic
Nate Wilkinson
Seth Kingston
John Young


# Pin Assignments
PC4:    USART3_TX
PC5:    USART3_RX
PC8:    Orange LED
PC9:    Green LED
PA0:    Button
PB5:    MOTOR Y - PWM DIR A (IN1)
PB6:    MOTOR Y - PWM DIR B (IN2)
PA4:    MOTOR Y - PWM Enable (TIMER 14 CH1)
PC6:    MOTOR Y - Encoder A Output (TIMER 3 CH1 AND CH2)
PC7:    MOTOR Y - Encoder B Output (TIMER 3 CH1 AND CH2)
PB3:    MOTOR X - PWM DIR A (IN1)
PB4:    MOTOR X - PWM DIR B (IN2)
TBA:    MOTOR X - PWM Enable (TIMER X CH1)
PA8:    MOTOR X - Encoder A Output (TIMER 1 CH1 AND CH2)
PA9:    MOTOR X - Encoder B Output (TIMER 1 CH1 AND CH2)

DON'T USE THESE PINS THEY DON'T WORK!!!: PA2, PA3, PA6, PA7, PB0, PB1


AVAILABLE TIMERS: TIM2, TIM15, TIM16, TIM17



VERIFIED GPIO PINS (FOR GENERAL PURPOSE OUTPUT MODE)
WORKING:
PA0 // But typically reserved for button
PA1 // NOT WORKING FOR AF5 TIM15_CH1N
PA4
PA5
PA8
PA9
PA10
PA15
PB2
PB3
PB4
PB5
PB6
PB7
PB8   // NOT WORKING FOR AF2 TIM16_CH1
PB9   //
PB10  
PB11
PB12
PB13
PB14  // NOT WORKING FOR AF1 TIM15_CH1
PB15  // NOT WORKING FOR AF3 TIM15_CH1N
PC0
PC3
PC4
PC5
PC6
PC7
PC8
PC9
PC10
PC11
PC12
PC13
PC14
PC15
PF0
PF1
PD2

NOT WORKING:
PA2
PA3
PA6
PA7
PA11
PA12
PA13
PA14
PB0
PB1
PC1
PC2
