#include "cavebot_user.h"

#include <stdbool.h>

#include "aether.h"
#include "cavetalk.h"
#include "spi.h"

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"
#include "bsp_motor.h"
#include "bsp_pwm.h"
#include "bsp_pwm_user.h"
#include "bsp_timer_user.h"
#include "bsp_servo.h"
#include "bsp_spi_user.h"

#include "lsm6dsv16x.h"
#include "rgbw.h"

#include "cavebot.h"
#include "comms.h"
#include "fault_handler.h"
#include "fsm.h"
#include "scheduler.h"

static const char *const kCavebotUser_LogTag = "CAVEBOT USER";

static Lsm6dsv16x_Context_t kCavebotUser_Lsm6dsv16x = LSM6DSV16X_CONTEXT(BSP_SPI_USER_0, BSP_GPIO_USER_PIN_IMU_CS);

BspMotor_Handle_t CavebotUser_Motors[CAVEBOT_USER_MOTOR_MAX] = {
    [CAVEBOT_USER_MOTOR_0] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 26.0,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_1] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_3,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_4,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 26.0,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_2] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_2,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 26.0,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_3] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_3,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 26.0,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    }
};

BspEncoderUser_Timer_t CavebotUser_Encoders[CAVEBOT_USER_ENCODER_MAX] = {
    [CAVEBOT_USER_ENCODER_0] = BSP_ENCODER_USER_TIMER_0,
    [CAVEBOT_USER_ENCODER_1] = BSP_ENCODER_USER_TIMER_1,
    [CAVEBOT_USER_ENCODER_2] = BSP_ENCODER_USER_TIMER_2,
    [CAVEBOT_USER_ENCODER_3] = BSP_ENCODER_USER_TIMER_3
};

Rgbw_Handle_t CavebotUser_Rgbw = {
    .pins = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_USER_PIN_STATUS_LED_RED,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_USER_PIN_STATUS_LED_GREEN,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_USER_PIN_STATUS_LED_BLUE,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_USER_PIN_STATUS_LED_WHITE,
    },
};

Accelerometer_Handle_t CavebotUser_Accelerometer = LSM6DSV16X_ACCELEROMETER_HANDLE(kCavebotUser_Lsm6dsv16x);
Gyroscope_Handle_t     CavebotUser_Gyroscope     = LSM6DSV16X_GYROSCOPE_HANDLE(kCavebotUser_Lsm6dsv16x);

static void CavebotUser_ImuTask(void);
static void CavebotUser_EncoderTask(void);
static void CavebotUser_Task(void);
static void CavebotUser_CommsTask(void);
static void CavebotUser_ExitInitialize(void);

static void CavebotUser_OnArm(void);
static void CavebotUser_OnDisarm(void);
static Fsm_State_t *CavebotUser_UpdateState(void);

static Fsm_t             CavebotUser_Fsm;
static const char *const CavebotUser_FsmName                   = "CAVEBOT USER  FSM";
static Fsm_State_t       CavebotUser_States[CAVEBOT_STATE_MAX] = {
    [CAVEBOT_STATE_INITIALIZE] = {
        .name   = "INITIALIZE",
        .enter  = NULL,
        .update = CavebotUser_UpdateState,
        .exit   = CavebotUser_ExitInitialize,
    },
    [CAVEBOT_STATE_READY] = {
        .name   = "READY",
        .enter  = NULL,
        .update = CavebotUser_UpdateState,
        .exit   = NULL,
    },
    [CAVEBOT_STATE_MANUAL] = {
        .name   = "MANUAL",
        .enter  = CavebotUser_OnArm,
        .update = CavebotUser_UpdateState,
        .exit   = CavebotUser_OnDisarm,
    },
    [CAVEBOT_STATE_AUTO] = {
        .name   = "AUTO",
        .enter  = CavebotUser_OnArm,
        .update = CavebotUser_UpdateState,
        .exit   = CavebotUser_OnDisarm,
    },
    [CAVEBOT_STATE_FAILED] = {
        .name   = "FAILED",
        .enter  = NULL,
        .update = CavebotUser_UpdateState,
        .exit   = NULL,
    },
};

bool CavebotUser_Initialize(void)
{
    bool initialized = false;

    Bsp_Error_t error = Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_YELLOW);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to set RGBW color with error %s", Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_RGBW, error);
        initialized = false;
    }

    error = Accelerometer_Initialize(&CavebotUser_Accelerometer);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize accelerometer with error %s", Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_ACCELEROMETER, error);
        initialized = false;
    }

    error = Gyroscope_Initialize(&CavebotUser_Gyroscope);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize gyroscope with error %s", Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_GYROSCOPE, error);
        initialized = false;
    }

    error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_0);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize encoder %d with error %s", BSP_ENCODER_USER_TIMER_0, Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        initialized = false;
    }

    error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_1);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize encoder %d with error %s", BSP_ENCODER_USER_TIMER_1, Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        initialized = false;
    }

    error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_2);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize encoder %d with error %s", BSP_ENCODER_USER_TIMER_2, Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        initialized = false;
    }

    error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_3);
    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize encoder %d with error %s", BSP_ENCODER_USER_TIMER_3, Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        initialized = false;
    }

    if (!Fsm_Initialize(&CavebotUser_Fsm, &CavebotUser_States[CAVEBOT_STATE_INITIALIZE], CavebotUser_FsmName))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to initialize FSM");
        initialized = false;
    }

    if (!Scheduler_AddTask(CavebotUser_ImuTask, 2U))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to add IMU task to scheduler");
        initialized = false;
    }

    if (!Scheduler_AddTask(CavebotUser_EncoderTask, 40U))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to add encoder task to scheduler");
        initialized = false;
    }

    if (!Scheduler_AddTask(CavebotUser_Task, 800U))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to add board task to scheduler");
        initialized = false;
    }

    if (!Scheduler_AddTask(CavebotUser_CommsTask, 40U))
    {
        BSP_LOGGER_LOG_ERROR(kCavebotUser_LogTag, "Failed to add telemetry task to scheduler");
        initialized = false;
    }

    return initialized;
}

static void CavebotUser_ImuTask(void)
{
    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_ACCELEROMETER))
    {
        const Bsp_Error_t error = Accelerometer_Read(&CavebotUser_Accelerometer);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_ACCELEROMETER, error);
        }
    }

    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_GYROSCOPE))
    {
        const Bsp_Error_t error = Gyroscope_Read(&CavebotUser_Gyroscope);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_GYROSCOPE, error);
        }
    }
}

static void CavebotUser_EncoderTask(void)
{
    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_ENCODER))
    {
        Bsp_Error_t error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        }

        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_1);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        }

        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_2);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        }

        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_3);
        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_ENCODER, error);
        }
    }
}

void CavebotUser_Task(void)
{
    Fsm_Update(&CavebotUser_Fsm);
}

static void CavebotUser_CommsTask(void)
{
    const cavetalk_Acceleration acceleration = {
        .x_meters_per_second_squared = CavebotUser_Accelerometer.reading.x,
        .y_meters_per_second_squared = CavebotUser_Accelerometer.reading.y,
        .z_meters_per_second_squared = CavebotUser_Accelerometer.reading.z,
    };
    Comms_SpeakAcceleration(&acceleration);

    const cavetalk_Gyroscope gyroscope = {
        .roll_radians_per_second  = CavebotUser_Gyroscope.reading.x,
        .pitch_radians_per_second = CavebotUser_Gyroscope.reading.y,
        .yaw_radians_per_second   = CavebotUser_Gyroscope.reading.z,
    };
    Comms_SpeakGyroscope(&gyroscope);

    cavetalk_Encoder encoders[BSP_ENCODER_USER_TIMER_MAX] = {
        [BSP_ENCODER_USER_TIMER_0] = {
            .pulses                  = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses,
            .rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].angular_rate,
        },
        [BSP_ENCODER_USER_TIMER_1] = {
            .pulses                  = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses,
            .rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].angular_rate,
        },
        [BSP_ENCODER_USER_TIMER_2] = {
            .pulses                  = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses,
            .rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].angular_rate,
        },
        [BSP_ENCODER_USER_TIMER_3] = {
            .pulses                  = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses,
            .rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].angular_rate,
        },
    };
    Comms_SpeakEncoders(encoders, sizeof(encoders) / sizeof(encoders[0]));

    /* TODO CVW-22 faults */
}

static void CavebotUser_ExitInitialize(void)
{
    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_RGBW))
    {
        /* TODO CVW-50 handle RGBW LED errors */
        (void)Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_GREEN);
    }

    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_BUZZER))
    {
        /* TODO CVW-67 make all sounds non-block and add error handling */
        /* Initialization sound */
        BspPwm_Start(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
        BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1, 0.5);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_4, 31110);
        Bsp_Delay(100);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_4, 23333);
        Bsp_Delay(100);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_4, 15556);
        Bsp_Delay(100);
        BspPwm_Stop(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
    }
}

static void CavebotUser_OnArm(void)
{
    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_RGBW))
    {
        /* TODO CVW-50 handle RGBW LED errors */
        (void)Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_RED);
    }

    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_BUZZER))
    {
        /* TODO CVW-67 make all sounds non-block and add error handling */
        /* Arm sound */
        BspPwm_Start(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
        BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1, 0.5);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_4, 15556);
        Bsp_Delay(100);
        BspPwm_Stop(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
        Bsp_Delay(100);
        BspPwm_Start(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
        Bsp_Delay(100);
        BspPwm_Stop(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
    }
}

static void CavebotUser_OnDisarm(void)
{
    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_RGBW))
    {
        /* TODO CVW-50 handle RGBW LED errors */
        (void)Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_GREEN);
    }

    if (!FaultHandler_HasFault(FAULT_HANDLER_FAULT_BUZZER))
    {
        /* TODO CVW-67 make all sounds non-block and add error handling */
        /* Disarm sound */
        BspPwm_Start(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
        BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1, 0.5);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_4, 15556);
        Bsp_Delay(300);
        BspPwm_Stop(BSP_PWM_USER_TIMER_4, BSP_TIMER_CHANNEL_1);
    }
}

static Fsm_State_t *CavebotUser_UpdateState(void)
{
    return &CavebotUser_States[Cavebot_GetState()];
}