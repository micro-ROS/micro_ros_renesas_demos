#include <stdint.h>
#include "motor.h"
#include "hal_data.h"
#include "rm_motor_api.h"
#include "micro_ros_thread.h"

static const motor_instance_t *p_motor_instance;
static uint8_t g_motor_status;
static uint16_t g_chk_error;

#define NORMALIZE(val, min, max) (((val) < (min)) ? (min) : ((val) > (max)) ? (max) : (val))

/* --- Internal function prototypes --- */
static void motor_fsp_init(void);
static void software_init(void);

/***********************************************************************************************************************
 * Initialization
 ***********************************************************************************************************************/
void mtr_init(void)
{
    p_motor_instance = &g_motor_sensorless0;
    motor_fsp_init();
    software_init();

    // Reset internal state
    p_motor_instance->p_api->reset(p_motor_instance->p_ctrl);
}

/***********************************************************************************************************************
 * FSP initialization
 ***********************************************************************************************************************/
static void motor_fsp_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* Open motor instance */
    err = p_motor_instance->p_api->open(p_motor_instance->p_ctrl, p_motor_instance->p_cfg);
    assert(FSP_SUCCESS == err);

    /* Open POEG for protection (optional, keep it open) */
    err = R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);
    assert(FSP_SUCCESS == err);
}

/***********************************************************************************************************************
 * Software initialization
 ***********************************************************************************************************************/
static void software_init(void)
{
    g_motor_status = MOTOR_STATUS_STOP;
}

/***********************************************************************************************************************
 * Main motor control
 ***********************************************************************************************************************/
void process_motor_command(int rpm)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t status;
    err = p_motor_instance->p_api->statusGet(p_motor_instance->p_ctrl, &status);
    assert(FSP_SUCCESS == err);

    if (status == MOTOR_STATUS_ERROR)
    {
        // Reset on error
        err = p_motor_instance->p_api->reset(p_motor_instance->p_ctrl);
        assert(FSP_SUCCESS == err);
    }

    if (rpm == 0)
    {
        // stop the motor if it is running
        if (status == MOTOR_STATUS_RUN)
        {
            err = p_motor_instance->p_api->stop(p_motor_instance->p_ctrl);
            assert(FSP_SUCCESS == err);
        }

        return;
    }

    // Get the target RPM. Negative RPM values represent reverse rotation
    int target_rpm = rpm > 0 ? NORMALIZE(rpm, MIN_MTR_SPEED_RPM, MAX_MTR_SPEED_RPM)
                                : NORMALIZE(rpm, -MAX_MTR_SPEED_RPM, -MIN_MTR_SPEED_RPM);

    // Start the motor and set the rotation speed
    if (status == MOTOR_STATUS_STOP)
    {
        err = p_motor_instance->p_api->run(p_motor_instance->p_ctrl);
        assert(FSP_SUCCESS == err);
    }

    err = p_motor_instance->p_api->speedSet(p_motor_instance->p_ctrl, (float) target_rpm);
    assert(FSP_SUCCESS == err);
}

/***********************************************************************************************************************
 * Motor control callback
 * It is triggered by the motor sensoreless module during the speed control periodic interrupt.
 ***********************************************************************************************************************/
void mtr_callback_event(motor_callback_args_t * p_args)
{
    fsp_err_t err = FSP_SUCCESS;
    // Check for motor current errors
    if ((p_args->event == MOTOR_CALLBACK_EVENT_CURRENT_FORWARD) && (g_motor_status != MOTOR_STATUS_ERROR))
    {
        p_motor_instance->p_api->errorCheck(p_motor_instance->p_ctrl, &g_chk_error);
        assert(FSP_SUCCESS == err);
    }
}

/***********************************************************************************************************************
 * POEG Overcurrent Callback
 ***********************************************************************************************************************/
void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    fsp_err_t err = FSP_SUCCESS;
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
        err = p_motor_instance->p_api->errorSet(p_motor_instance->p_ctrl, MOTOR_ERROR_OVER_CURRENT_HW);
        assert(FSP_SUCCESS == err);
    }
}
