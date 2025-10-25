#include "bsp_gpio_user.h"

#include "main.h"

#include "bsp.h"

Bsp_Gpio_t BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MAX] = {
    [BSP_GPIO_USER_PIN_IMU_CS] = {
        .gpio_port = IMU_CS_GPIO_Port,
        .gpio_pin  = IMU_CS_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_IMU_INT1] = {
        .gpio_port = IMU_INT1_GPIO_Port,
        .gpio_pin  = IMU_INT1_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_IMU_INT2] = {
        .gpio_port = IMU_INT2_GPIO_Port,
        .gpio_pin  = IMU_INT2_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_0_SLEEP] = {
        .gpio_port = MOTOR_0_SLEEP_GPIO_Port,
        .gpio_pin  = MOTOR_0_SLEEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_1_SLEEP] = {
        .gpio_port = MOTOR_1_SLEEP_GPIO_Port,
        .gpio_pin  = MOTOR_1_SLEEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_2_SLEEP] = {
        .gpio_port = MOTOR_2_SLEEP_GPIO_Port,
        .gpio_pin  = MOTOR_2_SLEEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_3_SLEEP] = {
        .gpio_port = MOTOR_3_SLEEP_GPIO_Port,
        .gpio_pin  = MOTOR_3_SLEEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_0_FAULT] = {
        .gpio_port = MOTOR_0_FAULT_GPIO_Port,
        .gpio_pin  = MOTOR_0_FAULT_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_1_FAULT] = {
        .gpio_port = MOTOR_1_FAULT_GPIO_Port,
        .gpio_pin  = MOTOR_1_FAULT_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_2_FAULT] = {
        .gpio_port = MOTOR_2_FAULT_GPIO_Port,
        .gpio_pin  = MOTOR_2_FAULT_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_3_FAULT] = {
        .gpio_port = MOTOR_3_FAULT_GPIO_Port,
        .gpio_pin  = MOTOR_3_FAULT_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STATUS_LED_WHITE] = {
        .gpio_port = STATUS_LED_WHITE_GPIO_Port,
        .gpio_pin  = STATUS_LED_WHITE_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STATUS_LED_RED] = {
        .gpio_port = STATUS_LED_RED_GPIO_Port,
        .gpio_pin  = STATUS_LED_RED_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STATUS_LED_GREEN] = {
        .gpio_port = STATUS_LED_GREEN_GPIO_Port,
        .gpio_pin  = STATUS_LED_GREEN_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STATUS_LED_BLUE] = {
        .gpio_port = STATUS_LED_BLUE_GPIO_Port,
        .gpio_pin  = STATUS_LED_BLUE_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STEPPER_MOTOR_DIRECTION] = {
        .gpio_port = STEPPER_MOTOR_DIRECTION_GPIO_Port,
        .gpio_pin  = STEPPER_MOTOR_DIRECTION_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
    [BSP_GPIO_USER_PIN_STEPPER_MOTOR_STEP] = {
        .gpio_port = STEPPER_MOTOR_STEP_GPIO_Port,
        .gpio_pin  = STEPPER_MOTOR_STEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = {
            .function = NULL,
            .arg      = NULL,
        },
        .debounce = 0U,
        .previous = 0U,
    },
};

Bsp_Gpio_t *BspGpioUser_GetGpioHandle(const Bsp_GpioPin_t exti_pin)
{
    Bsp_Gpio_t *gpio = NULL;

    switch (exti_pin)
    {
    case IMU_INT1_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_IMU_INT1];
        break;
    case IMU_INT2_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_IMU_INT2];
        break;
    case MOTOR_0_FAULT_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MOTOR_0_FAULT];
        break;
    case MOTOR_1_FAULT_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MOTOR_1_FAULT];
        break;
    case MOTOR_2_FAULT_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MOTOR_2_FAULT];
        break;
    case MOTOR_3_FAULT_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MOTOR_3_FAULT];
        break;
    default:
        break;
    }

    return gpio;
}