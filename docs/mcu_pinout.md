# Rover Controller MCU Pinout

Revision 9

Based on STM32F407ZGTx

| Name                     | Function          | Pin                        |
| ------------------------ | ----------------- | -------------------------- |
| Motor 0 +                | PWM               | PE9                        |
| Motor 0 -                | PWM               | PE11                       |
| Motor 1 +                | PWM               | PE13                       |
| Motor 1 -                | PWM               | PE14                       |
| Motor 2 +                | PWM               | PC6                        |
| Motor 2 -                | PWM               | PC7                        |
| Motor 3 +                | PWM               | PC8                        |
| Motor 3 -                | PWM               | PC9                        |
| Steering Servo 0         | PWM               | PE5                        |
| Steering Servo 1         | PWM               | PE6                        |
| Steering Servo 2         | PWM               | PB14                       |
| Steering Servo 3         | PWM               | PB15                       |
| Camera Servo Pan         | PWM               | PF6                        |
| Camera Servo Tilt        | PWM               | PF7                        |
| Encoder 0 A              | Timer             | PA15                       |
| Encoder 0 B              | Timer             | PB3                        |
| Encoder 1 A              | Timer             | PB4                        |
| Encoder 1 B              | Timer             | PB5                        |
| Encoder 2 A              | Timer             | PD12                       |
| Encoder 2 B              | Timer             | PD13                       |
| Encoder 3 A              | Timer             | PA0-WKUP                   |
| Encoder 3 B              | Timer             | PA1                        |
| IMU SCL                  | SPI Clock         | PB10                       |
| IMU SDA                  | SPI PICO (MOSI)   | PC3                        |
| IMU SDO                  | SPI POCI (MISO)   | PC2                        |
| IMU CS                   | SPI CS            | PC13                       |
| IMU INT1                 | Digital Interrupt | PC14                       |
| IMU INT2                 | Digital Interrupt | PE0                        |
| General Fault LED        | Digital Output    | PF14                       |
| Motor Fault LED          | Digital Output    | PG0                        |
| Low Battery LED          | Digital Output    | PG2                        |
| Battery Charged LED      | Digital Output    | PG3                        |
| Jetson Comms LED         | Digital Output    | PE1                        |
| Jetson Fault LED         | Digital Output    | PF5                        |
| Wifi Status LED          | Digital Output    | PG10                       |
| IMU Status LED           | Digital Output    | PD0                        |
| Encoders Status LED      | Digital Output    | PE7                        |
| Board Temp LED           | Digital Output    | PE10                       |
| Board Humidity LED       | Digital Output    | PE12                       |
| Dust Sensor Status LED   | Digital Output    | PE8                        |
| LED Pod 0                | Digital Output    | PG4                        |
| LED Pod 1                | Digital Output    | PG5                        |
| LED Pod 2                | Digital Output    | PG6                        |
| Motor Sleep              | Digital Output    | PD14                       |
| Fan                      | Digital Output    | PD1                        |
| Start Button             | Digital Interrupt | PB2                        |
| Manual Mode Button       | Digital Interrupt | PF12                       |
| Headlights Enable Button | Digital Interrupt | PF13                       |
| Motor Enable Button      | Digital Interrupt | PG1                        |
| Gas Sensor DOUT          | Digital Interrupt | PD7                        |
| Motor 0 nFault           | Digital Input     | PF3                        |
| Motor 1 nFault           | Digital Input     | PF4                        |
| Motor 2 nFault           | Digital Input     | PF10                       |
| Motor 3 nFault           | Digital Input     | PG11                       |
| SWCLK                    | SWCLK             | PA14                       |
| SWDIO                    | SWDIO             | PA13                       |
| Reset                    | NRST              | NRST                       |
| SWO                      | SWO               | In Use (Encoder 0 B - PB3) |
| Gas Sensor AOUT          | ADC               | PA2                        |
| Temperature Sensor       | ADC               | PA3                        |
| Motor 0 current          | ADC               | PA4                        |
| Motor 1 current          | ADC               | PA5                        |
| Motor 2 current          | ADC               | PA6                        |
| Motor 3 current          | ADC               | PA7                        |
| I2C 1 SCL                | I2C SCL           | PB6                        |
| I2C 1 SDA                | I2C SDA           | PB7                        |
| I2C 2 SCL                | I2C SCL           | PF1                        |
| I2C 2 SDA                | I2C SDA           | PF0                        |
| Jetson UART TX           | UART TX           | PA9                        |
| Jetson UART RX           | UART RX           | PA10                       |
| Dust Sensor UART TX      | UART TX           | PG14                       |
| Dust Sensor UART RX      | UART RX           | PG9                        |
| Logging UART TX          | UART TX           | PD8                        |
| Logging UART RX          | UART RX           | PB11                       |
| HSE IN                   | HSE               | PH0                        |
| HSE OUT                  | HSE               | PH1                        |

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

- Interrupts in use on 0, 1, 2, 3, 4, 7, 10, 11, 12, 13, 14
  
  - Consider PD5, PD6, PD9 for remaining digital interrupts