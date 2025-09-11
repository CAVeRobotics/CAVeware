# CAVeBoard-Mini MCU Pinout

Revision 0

Based on STM32F407ZGTx

| Name              | Function          | Pin      |
| ----------------- | ----------------- | -------- |
| Motor 0 +         | PWM               | PE9      |
| Motor 0 -         | PWM               | PE11     |
| Motor 1 +         | PWM               | PE13     |
| Motor 1 -         | PWM               | PE14     |
| Motor 2 +         | PWM               | PC6      |
| Motor 2 -         | PWM               | PC7      |
| Motor 3 +         | PWM               | PC8      |
| Motor 3 -         | PWM               | PC9      |
| PWM 0             | PWM               | PE5      |
| PWM 1             | PWM               | PE6      |
| PWM 2             | PWM               | PF6      |
| PWM 3             | PWM               | PF7      |
| PWM 4             | PWM               | PB14     |
| PWM 5             | PWM               | PB15     |
| PWM 6             | PWM               | PF8      |
| PWM 7             | PWM               | PF9      |
| Encoder 0 A       | Timer             | PA15     |
| Encoder 0 B       | Timer             | PB3      |
| Encoder 1 A       | Timer             | PB4      |
| Encoder 1 B       | Timer             | PB5      |
| Encoder 2 A       | Timer             | PD12     |
| Encoder 2 B       | Timer             | PD13     |
| Encoder 3 A       | Timer             | PA0-WKUP |
| Encoder 3 B       | Timer             | PA1      |
| IMU SCL           | SPI Clock         | PB10     |
| IMU SDA           | SPI PICO (MOSI)   | PC3      |
| IMU SDO           | SPI POCI (MISO)   | PC2      |
| IMU CS            | SPI CS            | PD0      |
| IMU INT1          | Digital Interrupt | PE0      |
| IMU INT2          | Digital Interrupt | PE1      |
| Motor 0 nFault    | Digital Interrupt | PF3      |
| Motor 1 nFault    | Digital Interrupt | PF4      |
| Motor 2 nFault    | Digital Interrupt | PF10     |
| Motor 3 nFault    | Digital Interrupt | PG11     |
| IMU Status LED    | Digital Output    | PC13     |
| Motor 0 Sleep     | Digital Output    | PD14     |
| Motor 1 Sleep     | Digital Output    | PG2      |
| Motor 2 Sleep     | Digital Output    | PG3      |
| Motor 3 Sleep     | Digital Output    | PG4      |
| SWCLK             | SWCLK             | PA14     |
| SWDIO             | SWDIO             | PA13     |
| Reset             | NRST              | NRST     |
| Motor 0 current   | ADC               | PA4      |
| Motor 1 current   | ADC               | PA5      |
| Motor 2 current   | ADC               | PA6      |
| Motor 3 current   | ADC               | PB0      |
| Interface UART TX | UART TX           | PA9      |
| Interface UART RX | UART RX           | PA10     |
| Sensor UART TX    | UART TX           | PG14     |
| Sensor UART RX    | UART RX           | PG9      |
| Logging UART TX   | UART TX           | PD8      |
| Logging UART RX   | UART RX           | PB11     |
| HSE IN            | HSE               | PH0      |
| HSE OUT           | HSE               | PH1      |

## Timer Mapping

Encoders: TIM 2, TIM 3, TIM 4, TIM 5

Motors: TIM 1, TIM 8 (programmable dead times can be inserted)

Spare PWM: TIM9, TIM10, TIM11, TIM12, TIM 13, TIM 14
