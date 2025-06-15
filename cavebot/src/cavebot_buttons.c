#include "cavebot_buttons.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"
#include "bsp_tick.h"

#include "rover.h"

typedef struct
{
    BspGpioUser_Pin_t pin;
    Bsp_GpioState_t trigger_state;
    Bsp_Microsecond_t interval_min;
    Bsp_Microsecond_t interval_max;
    Bsp_GpioState_t previous_state;
    Bsp_Microsecond_t previous_tick;
    void (*on_press)(const BspGpioUser_Pin_t pin);
} CavebotButtons_Handle_t;

static const char *kCavebotButtons_LogTag = "BUTTONS";

static void CavebotButtons_OnPress(const Bsp_GpioPin_t pin);
static void CavebotButtons_HeadlightsCallback(const BspGpioUser_Pin_t pin);
static void CavebotButtons_StartCallback(const BspGpioUser_Pin_t pin);
static void CavebotButtons_EnableCallback(const BspGpioUser_Pin_t pin);

static CavebotButtons_Handle_t CavebotButtons_HandleTable[CAVEBOT_BUTTONS_BUTTON_MAX] = {
    [CAVEBOT_BUTTONS_BUTTON_HEADLIGHTS] = {
        .pin = BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE,
        .trigger_state = BSP_GPIO_STATE_SET,
        .interval_min = 0U, /* TODO SD-130 tune for normal button press */
        .interval_max = UINT64_MAX,
        .previous_state = BSP_GPIO_STATE_RESET,
        .previous_tick = 0U,
        .on_press = CavebotButtons_HeadlightsCallback,
    },
    [CAVEBOT_BUTTONS_BUTTON_START] = {
        .pin = BSP_GPIO_USER_PIN_START,
        .trigger_state = BSP_GPIO_STATE_SET,
        .interval_min = 0U,
        .interval_max = UINT64_MAX,
        .previous_state = BSP_GPIO_STATE_RESET,
        .previous_tick = 0U,
        .on_press = CavebotButtons_StartCallback,
    },
    [CAVEBOT_BUTTONS_BUTTON_ENABLE] = {
        .pin = BSP_GPIO_USER_PIN_ENABLE,
        .trigger_state = BSP_GPIO_STATE_SET,
        .interval_min = 0U,
        .interval_max = UINT64_MAX,
        .previous_state = BSP_GPIO_STATE_RESET,
        .previous_tick = 0U,
        .on_press = CavebotButtons_EnableCallback,
    }};

Bsp_Error_t CavebotButtons_Enable(const CavebotButtons_Button_t button)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (button < CAVEBOT_BUTTONS_BUTTON_MAX)
    {
        error = BspGpio_RegisterCallback(CavebotButtons_HandleTable[button].pin, CavebotButtons_OnPress);
    }

    return error;
}

Bsp_Error_t CavebotButtons_Disable(const CavebotButtons_Button_t button)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (button < CAVEBOT_BUTTONS_BUTTON_MAX)
    {
        error = BspGpio_RegisterCallback(CavebotButtons_HandleTable[button].pin, NULL);
    }

    return error;
}

static void CavebotButtons_OnPress(const Bsp_GpioPin_t pin)
{
    CavebotButtons_Button_t button = CAVEBOT_BUTTONS_BUTTON_MAX;
    Bsp_GpioState_t gpio_state = BSP_GPIO_STATE_RESET;
    Bsp_Microsecond_t tick = BspTick_GetMicroseconds();

    if (pin == BspGpioUser_HandleTable[CavebotButtons_HandleTable[CAVEBOT_BUTTONS_BUTTON_HEADLIGHTS].pin].gpio_pin)
    {
        button = CAVEBOT_BUTTONS_BUTTON_HEADLIGHTS;
    }
    else if (pin == BspGpioUser_HandleTable[CavebotButtons_HandleTable[CAVEBOT_BUTTONS_BUTTON_START].pin].gpio_pin)
    {
        button = CAVEBOT_BUTTONS_BUTTON_START;
    }
    else if (pin == BspGpioUser_HandleTable[CavebotButtons_HandleTable[CAVEBOT_BUTTONS_BUTTON_ENABLE].pin].gpio_pin)
    {
        button = CAVEBOT_BUTTONS_BUTTON_ENABLE;
    }
    /* TODO SD-130 add other buttons here */

    if ((button >= CAVEBOT_BUTTONS_BUTTON_MAX) || (BSP_ERROR_NONE != BspGpio_Read(CavebotButtons_HandleTable[button].pin, &gpio_state)))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotButtons_LogTag, "Failed to read GPIO");
    }
    else if (gpio_state == CavebotButtons_HandleTable[button].trigger_state)
    {
        CavebotButtons_HandleTable[button].previous_state = gpio_state;
        CavebotButtons_HandleTable[button].previous_tick = tick;
    }
    else if ((CavebotButtons_HandleTable[button].previous_state == CavebotButtons_HandleTable[button].trigger_state) &&
             ((tick - CavebotButtons_HandleTable[button].previous_tick) >= CavebotButtons_HandleTable[button].interval_min) &&
             ((tick - CavebotButtons_HandleTable[button].previous_tick) <= CavebotButtons_HandleTable[button].interval_max))
    {
        CavebotButtons_HandleTable[button].previous_state = gpio_state;

        if (NULL != CavebotButtons_HandleTable[button].on_press)
        {
            CavebotButtons_HandleTable[button].on_press(pin);
        }
    }
}

static void CavebotButtons_HeadlightsCallback(const BspGpioUser_Pin_t pin)
{
    BSP_UNUSED(pin);

    (void)BspGpio_Toggle(BSP_GPIO_USER_PIN_HEADLIGHTS_0);
    (void)BspGpio_Toggle(BSP_GPIO_USER_PIN_HEADLIGHTS_1);
    (void)BspGpio_Toggle(BSP_GPIO_USER_PIN_HEADLIGHTS_2);

    BSP_LOGGER_LOG_INFO(kCavebotButtons_LogTag, "Toggle headlights");
}

static void CavebotButtons_StartCallback(const BspGpioUser_Pin_t pin)
{
    BSP_UNUSED(pin);

    BSP_LOGGER_LOG_DEBUG(kCavebotButtons_LogTag, "Start button pressed");

    if (Rover_IsArmed())
    {
        (void)Rover_Dearm();
    }
    else
    {
        (void)Rover_Arm();
    }
}

static void CavebotButtons_EnableCallback(const BspGpioUser_Pin_t pin)
{
    BSP_UNUSED(pin);

    BSP_LOGGER_LOG_DEBUG(kCavebotButtons_LogTag, "Enable button pressed");
}