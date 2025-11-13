# CAVeBoard-Mini MCU Pinout

Revision 5

Based on STM32F407ZGTx

| Name                    | Function          | Pin      | Number |
| ----------------------- | ----------------- | -------- | ------ |
| Motor 0 +               | PWM               | PE9      | B43    |
| Motor 0 -               | PWM               | PE11     | B42    |
| Motor 1 +               | PWM               | PE13     | B41    |
| Motor 1 -               | PWM               | PE14     | B12    |
| Motor 2 +               | PWM               | PC6      | T35    |
| Motor 2 -               | PWM               | PC7      | T6     |
| Motor 3 +               | PWM               | PC8      | T36    |
| Motor 3 -               | PWM               | PC9      | T7     |
| PWM 0                   | PWM               | PE5      | T53    |
| PWM 1                   | PWM               | PE6      | T25    |
| PWM 2                   | PWM               | PF6      | T58    |
| PWM 3                   | PWM               | PF7      | B55    |
| PWM 4                   | PWM               | PB14     | B35    |
| PWM 5                   | PWM               | PB15     | B6     |
| PWM 6                   | PWM               | PF8      | B26    |
| Buzzer                  | PWM               | PF9      | B56    |
| Encoder 0 A             | Timer             | PA15     | T54    |
| Encoder 0 B             | Timer             | PB3      | T17    |
| Encoder 1 A             | Timer             | PB4      | T46    |
| Encoder 1 B             | Timer             | PB5      | T18    |
| Encoder 2 A             | Timer             | PD12     | B32    |
| Encoder 2 B             | Timer             | PD13     | B3     |
| Encoder 3 A             | Timer             | PA0-WKUP | B25    |
| Encoder 3 B             | Timer             | PA1      | B54    |
| IMU SCL                 | SPI Clock         | PB10     | B11    |
| IMU SDA                 | SPI PICO (MOSI)   | PC3      | B29    |
| IMU SDO                 | SPI POCI (MISO)   | PC2      | B58    |
| IMU CS                  | SPI CS            | PD0      | T11    |
| IMU INT1                | Digital Interrupt | PE0      | T21    |
| IMU INT2                | Digital Interrupt | PE1      | T49    |
| Motor 0 nFault          | Digital Interrupt | PD9      | B5     |
| Motor 1 nFault          | Digital Interrupt | PF4      | T57    |
| Motor 2 nFault          | Digital Interrupt | PD10     | B33    |
| Motor 3 nFault          | Digital Interrupt | PG11     | T16    |
| Status LED White        | Digital Output    | PE12     | B13    |
| Status LED Red          | Digital Output    | PE10     | B14    |
| Status LED Green        | Digital Output    | PE7      | B15    |
| Status LED Blue         | Digital Output    | PE8      | B44    |
| Motor 0 Sleep           | Digital Output    | PD14     | B8     |
| Motor 1 Sleep           | Digital Output    | PG2      | B9     |
| Motor 2 Sleep           | Digital Output    | PG3      | B38    |
| Motor 3 Sleep           | Digital Output    | PG4      | B10    |
| Stepper Motor Direction | Digital Output    | PF3      | T28    |
| Stepper Motor Step      | Digital Output    | PF5      | T29    |
| SWCLK                   | SWCLK             | PA14     | N/A    |
| SWDIO                   | SWDIO             | PA13     | N/A    |
| Reset                   | NRST              | NRST     | N/A    |
| Motor 0 current         | ADC               | PA4      | B50    |
| Motor 1 current         | ADC               | PA5      | B20    |
| Motor 2 current         | ADC               | PA6      | B49    |
| Motor 3 current         | ADC               | PB0      | B22    |
| Interface UART TX       | UART TX           | PA9      | T8     |
| Interface UART RX       | UART RX           | PA10     | T37    |
| Logging UART TX         | UART TX           | PG14     | T50    |
| Logging UART RX         | UART RX           | PG9      | T15    |
| HSE IN                  | HSE               | PH0      | N/A    |
| HSE OUT                 | HSE               | PH1      | N/A    |

## Timer Mapping

Encoders: TIM 2, TIM 3, TIM 4, TIM 5

Motors: TIM 1, TIM 8 (programmable dead times can be inserted)

Spare PWM: TIM9, TIM10, TIM11, TIM12, TIM 13, TIM 14
