# Rover Controller MCU Pinout

Based on STM32F407ZGTx

| Name              | Function        | Pin                        |
| ----------------- | --------------- | -------------------------- |
| Motor 0 +         | PWM             | PE9                        |
| Motor 0 -         | PWM             | PE11                       |
| Motor 1 +         | PWM             | PE13                       |
| Motor 1 -         | PWM             | PE14                       |
| Motor 2 +         | PWM             | PC6                        |
| Motor 2 -         | PWM             | PC7                        |
| Motor 3 +         | PWM             | PC8                        |
| Motor 3 -         | PWM             | PC9                        |
| Steering Servo 0  | PWM             | PE5                        |
| Steering Servo 1  | PWM             | PE6                        |
| Steering Servo 2  | PWM             | PB14                       |
| Steering Servo 3  | PWM             | PB15                       |
| Camera Servo Pan  | PWM             | PF6                        |
| Camera Servo Tilt | PWM             | PF7                        |
| Encoder 0 A       | Timer           | PA5                        |
| Encoder 0 B       | Timer           | PB3                        |
| Encoder 1 A       | Timer           | PA6                        |
| Encoder 1 B       | Timer           | PA7                        |
| Encoder 2 A       | Timer           | PD12                       |
| Encoder 2 B       | Timer           | PD13                       |
| Encoder 3 A       | Timer           | PA0-WKUP                   |
| Encoder 3 B       | Timer           | PA1                        |
| IMU SCL           | SPI Clock       | PB10                       |
| IMU SDA           | SPI PICO (MOSI) | PC3                        |
| IMU SDO           | SPI POCI (MISO) | PC2                        |
| IMU CS            | SPI CS          | PC13                       |
| IMU INT1          | Interrupt       | PC14                       |
| IMU INT2          | Interrupt       | PH0                        |
| LED 0             | Digital Output  | PF14                       |
| LED 1             | Digital Output  | PG0                        |
| LED 2             | Digital Output  | PG2                        |
| LED 3             | Digital Output  | PG3                        |
| Button 0          | Digital Input   | PB2                        |
| Button 1          | Digital Input   | PF12                       |
| Button 2          | Digital Input   | PF13                       |
| Button 3          | Digital Input   | PG1                        |
| UART TX           | UART TX         | PA9                        |
| UART RX           | UART RX         | PA10                       |
| LED Pod 0         | Digital Output  | PG4                        |
| LED Pod 1         | Digital Output  | PG5                        |
| LED Pod 2         | Digital Output  | PG6                        |
| SWCLK             | SWCLK           | PA14                       |
| SWDIO             | SWDIO           | PA13                       |
| Reset             | NRST            | NRST                       |
| SWO               | SWO             | In Use (Encoder 0 B - PB3) |
| ADC 1 IN 2        | ADC             | PA2                        |
| ADC 1 IN 3        | ADC             | PA3                        |
| ADC 1 IN 4        | ADC             | PA4                        |
| ADC 1 IN 5        | ADC             | PA5                        |
| I2C 1 SCL         | I2C SCL         | PB6                        |
| I2C 1 SDA         | I2C SDA         | PB7                        |
| I2C 2 SCL         | I2C SCL         | PF1                        |
| I2C SDA           | I2C SDA         | PF0                        |
| BMS               | ??              | ??                         |

### OSHWA SPI Naming Conventions

Some manufacturers are using the following replacement names for SPI pins.

| Obsolete Name | Replacement Name |
| ------------- | ---------------- |
| Master        | Controller       |
| Slave         | Peripheral       |
| MISO          | POCI             |
| MOSI          | PICO             |
| SS            | CS               |

## Timer Mapping

Encoders: TIM 2, TIM 3, TIM 4, TIM 5

Motors: TIM 1, TIM 8 (programmable dead times can be inserted)

Steering servos: TIM 9, TIM 12

Camera servos: TIM10, TIM 11

Spare PWM: TIM 13, TIM 14

## Notes

- Use advanced timers to drive motors so programmable dead times can be inserted into the PWM signals

- Consider adding jumpers or switches to physically enable/disable motors

- Disable remaining channels when timer in Encoder Mode (timers only have one counter register), i.e. one timer per encoder

- Break out channels from TIM10, TIM 11, TIM 13, and TIM 14 to block of four PWM headers
  
  - Camera servos will use two

- Consider breaking out several more GPIO
  
  - Any unused GPIO remaining can be used

- Break out BOOT0 pin too?
  
  - Maybe configure with a jumper
