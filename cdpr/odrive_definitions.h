#pragma once

// ODrive.GpioMode
typedef enum {
	GPIO_MODE_DIGITAL = 0,
	GPIO_MODE_DIGITAL_PULL_UP,
	GPIO_MODE_DIGITAL_PULL_DOWN,
	GPIO_MODE_ANALOG_IN,
	GPIO_MODE_UART_A,
	GPIO_MODE_UART_B,
	GPIO_MODE_UART_C,
	GPIO_MODE_CAN_A,
	GPIO_MODE_I2C_A,
	GPIO_MODE_SPI_A,
	GPIO_MODE_PWM,
	GPIO_MODE_ENC0,
	GPIO_MODE_ENC1,
	GPIO_MODE_ENC2,
	GPIO_MODE_MECH_BRAKE,
	GPIO_MODE_STATUS,
	GPIO_MODE_BRAKE_RES,
	GPIO_MODE_AUTO
} odrive_gpio_mode;

// ODrive.StreamProtocolType
typedef enum {
	STREAM_PROTOCOL_TYPE_FIBRE = 0,
	STREAM_PROTOCOL_TYPE_ASCII,
	STREAM_PROTOCOL_TYPE_STDOUT,
	STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT,
	STREAM_PROTOCOL_TYPE_OTHER
} odrive_stream_protocol_type;

// ODrive.Axis.AxisState
typedef enum {
	AXIS_STATE_UNDEFINED = 0,
	AXIS_STATE_IDLE,
	AXIS_STATE_STARTUP_SEQUENCE,
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
	AXIS_STATE_MOTOR_CALIBRATION,
	AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
	AXIS_STATE_CLOSED_LOOP_CONTROL,
	AXIS_STATE_LOCKIN_SPIN,
	AXIS_STATE_ENCODER_DIR_FIND,
	AXIS_STATE_HOMING,
	AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
	AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION
} axis_states;

// ODrive.Controller.ControlMode
typedef enum {
	CONTROL_MODE_VOLTAGE_CONTROL = 0,
	CONTROL_MODE_TORQUE_CONTROL,
	CONTROL_MODE_VELOCITY_CONTROL,
	CONTROL_MODE_POSITION_CONTROL
} odrive_control_mode;

// ODrive.ComponentStatus
typedef enum {
	COMPONENT_STATUS_NOMINAL = 0,
	COMPONENT_STATUS_NO_RESPONSE,
	COMPONENT_STATUS_INVALID_RESPONSE_LENGTH,
	COMPONENT_STATUS_PARITY_MISMATCH,
	COMPONENT_STATUS_ILLEGAL_HALL_STATE,
	COMPONENT_STATUS_POLARITY_NOT_CALIBRATED,
	COMPONENT_STATUS_PHASES_NOT_CALIBRATED,
	COMPONENT_STATUS_NUMERICAL_ERROR,
	COMPONENT_STATUS_MISSING_INPUT,
	COMPONENT_STATUS_RELATIVE_MODE,
	COMPONENT_STATUS_UNCONFIGURED,
	COMPONENT_STATUS_OVERSPEED,
	COMPONENT_STATUS_INDEX_NOT_FOUND,
	COMPONENT_STATUS_BAD_CONFIG,
	COMPONENT_STATUS_NOT_ENABLED,
	COMPONENT_STATUS_SPINOUT_DETECTED
} odrive_component_status;

// ODrive.ProcedureResult
typedef enum {
	PROCEDURE_RESULT_SUCCESS = 0,
	PROCEDURE_RESULT_BUSY,
	PROCEDURE_RESULT_CANCELLED,
	PROCEDURE_RESULT_DISARMED,
	PROCEDURE_RESULT_NO_RESPONSE,
	PROCEDURE_RESULT_POLE_PAIR_CPR_MISMATCH,
	PROCEDURE_RESULT_PHASE_RESISTANCE_OUT_OF_RANGE,
	PROCEDURE_RESULT_PHASE_INDUCTANCE_OUT_OF_RANGE,
	PROCEDURE_RESULT_UNBALANCED_PHASES,
	PROCEDURE_RESULT_INVALID_MOTOR_TYPE,
	PROCEDURE_RESULT_ILLEGAL_HALL_STATE,
	PROCEDURE_RESULT_TIMEOUT,
	PROCEDURE_RESULT_HOMING_WITHOUT_ENDSTOP,
	PROCEDURE_RESULT_INVALID_STATE,
	PROCEDURE_RESULT_NOT_CALIBRATED,
	PROCEDURE_RESULT_NOT_CONVERGING
} odrive_procedure_result;

// ODrive.EncoderId
typedef enum {
	ENCODER_ID_NONE = 0,
	ENCODER_ID_INC_ENCODER0,
	ENCODER_ID_INC_ENCODER1,
	ENCODER_ID_INC_ENCODER2,
	ENCODER_ID_SENSORLESS_ESTIMATOR,
	ENCODER_ID_SPI_ENCODER0,
	ENCODER_ID_SPI_ENCODER1,
	ENCODER_ID_SPI_ENCODER2,
	ENCODER_ID_HALL_ENCODER0,
	ENCODER_ID_HALL_ENCODER1,
	ENCODER_ID_RS485_ENCODER0,
	ENCODER_ID_RS485_ENCODER1,
	ENCODER_ID_RS485_ENCODER2,
	ENCODER_ID_ONBOARD_ENCODER0,
	ENCODER_ID_ONBOARD_ENCODER1
} odrive_encoder_id;

// ODrive.SpiEncoderMode
typedef enum {
	SPI_ENCODER_MODE_DISABLED = 0,
	SPI_ENCODER_MODE_RLS,
	SPI_ENCODER_MODE_AMS,
	SPI_ENCODER_MODE_CUI,
	SPI_ENCODER_MODE_AEAT,
	SPI_ENCODER_MODE_MA732,
	SPI_ENCODER_MODE_TLE,
	SPI_ENCODER_MODE_BISSC
} odrive_spi_encoder_mode;

// ODrive.Rs485EncoderMode
typedef enum {
	RS485_ENCODER_MODE_DISABLED = 0,
	RS485_ENCODER_MODE_AMT21_POLLING,
	RS485_ENCODER_MODE_AMT21_EVENT_DRIVEN,
	RS485_ENCODER_MODE_MBS,
	RS485_ENCODER_MODE_ODRIVE_OA1
} odrive_rs485_encoder_mode;

// ODrive.Controller.InputMode
typedef enum {
	INPUT_MODE_INACTIVE = 0,
	INPUT_MODE_PASSTHROUGH,
	INPUT_MODE_VEL_RAMP,
	INPUT_MODE_POS_FILTER,
	INPUT_MODE_MIX_CHANNELS,
	INPUT_MODE_TRAP_TRAJ,
	INPUT_MODE_TORQUE_RAMP,
	INPUT_MODE_MIRROR,
	INPUT_MODE_TUNING
} odrive_input_mode;

// ODrive.MotorType
typedef enum {
	MOTOR_TYPE_HIGH_CURRENT = 0,
	MOTOR_TYPE_GIMBAL,
	MOTOR_TYPE_ACIM
} odrive_motor_type;

// ODrive.Error
typedef enum {
	ODRIVE_ERROR_NONE                      = 0x00000000,
	ODRIVE_ERROR_CONTROL_ITERATION_MISSED  = 0x00000001,
	ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE      = 0x00000002,
	ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE       = 0x00000004,
	ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT = 0x00000008,
	ODRIVE_ERROR_DC_BUS_OVER_CURRENT       = 0x00000010,
	ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION  = 0x00000020,
	ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN      = 0x00000040,
	ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE  = 0x00000080,
} odrive_error;