#ifndef SERVO_H_
#define SERVO_H_

#include "stdio.h"	
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "mLog.h"
#include "rs485.h"
#include "string.h"
#include "DynamixelDriver.h"

#define REC_BUFFER_LEN 32
#define SERVO_MAX_PARAMS (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US    75
#define REC_WAIT_PARAMS_US   (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES 200

#define SERVO_INSTRUCTION_ERROR   (1 << 6)
#define SERVO_OVERLOAD_ERROR      (1 << 5)
#define SERVO_CHECKSUM_ERROR      (1 << 4)
#define SERVO_RANGE_ERROR         (1 << 3)
#define SERVO_OVERHEAT_ERROR      (1 << 2)
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)
#define SERVO_INPUT_VOLTAGE_ERROR (1)


enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
};

typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;

typedef struct ServoInstruction
{
	uint8_t header1;
	uint8_t header2;
	uint8_t id;
  uint8_t length;
	uint8_t command;
  uint8_t params[SERVO_MAX_PARAMS];
  uint8_t checksum;
} ServoInstruction;

class Servo{
	public:
		Servo(u32 baudrate=57600){
			m_baudrate=baudrate;
			delay_init();
			//memset(m_info, '\0', MAX_INFO_SIZE);
		}
		void OpenPort(){
			RS485_Init(m_baudrate);
		}
		
		int getTemperature(const uint8_t servoId);
		
		bool torque(uint8_t servoId, uint8_t onoff);
		bool torqueOn(uint8_t servoId);
		bool torqueOff(uint8_t servoId);
		
		bool changeID(uint8_t servoId, uint8_t new_id);
		bool changeBaudrate(uint8_t servoId, uint32_t new_baudrate);
		bool changeProtocolVersion(uint8_t servoId, uint8_t version); // TODO
		
		bool led(uint8_t servoId, int32_t onoff);
		bool ledOn(uint8_t servoId);
		bool ledOff(uint8_t servoId);
		
		bool setNormalDirection(uint8_t servoId);
		bool setReverseDirection(uint8_t servoId);

		bool setSecondaryID(uint8_t servoId, uint8_t secondary_id);

//		bool setCurrentControlMode(uint8_t servoId);
//		bool setTorqueControlMode(uint8_t servoId);
//		bool setVelocityControlMode(uint8_t servoId);  
//		bool setPositionControlMode(uint8_t servoId);  
//		bool setExtendedPositionControlMode(uint8_t servoId);
//		bool setMultiTurnControlMode(uint8_t servoId);
//		bool setCurrentBasedPositionControlMode(uint8_t servoId);
//		bool setPWMControlMode(uint8_t servoId);

		/*
		Value	Meaning
		0	Turn off the torque mode. Executes Joint mode or Wheel mode
		1	Turn on the torque mode. Cannot control the position or moving speed but only Torque
		When Torque Control Mode Enable is 1, DYNAMIXEL behaves like the followings

		DYNAMIXEL does not control position or velocity.
		DYNAMIXEL is controlled by the Goal Torque value.
		DYNAMIXEL does not affected by the Goal Position(30), Moving Speed(32) values.
		Since position/moving speed is not controlled, DYNAMIXEL behaves as if it is in the wheel mode.
		*/
		/*
		The angle limit allows the motion to be restrained. The range and the unit of the value is the same as Goal Position(30).

		CW Angle Limit: the minimum value of Goal Position(30)
		CCW Angle Limit: the maximum value of Goal Position(30)
		The following three modes can be set pursuant to the value of CW and CCW.
		Operation Type	CW / CCW
		Wheel Mode	Condition that both CW and CCW are 0
		Joint Mode	Neither CW or CCW are 0
		(Conditions excluding Wheel Mode and Multi-turn Mode)
		Multi-turn Mode	Condition that Both CW and CCW are 4095
		The wheel mode can be used to wheel-type operation robots since motors of the robots spin infinitely.
		The joint mode can be used to multi-joints robot since the robots can be controlled with specific angles. 
		Multi-turn mode allows joint mode control over multiple rotations (Position range : -28,672 ~ 28,672)
		*/
		bool setOperatingMode(uint8_t servoId, uint8_t index);

		bool jointMode(uint8_t servoId, int32_t velocity = 0, int32_t acceleration = 0);
		bool wheelMode(uint8_t servoId, int32_t acceleration = 0);
		// true -- jointMode false -- wheelMode
		bool isJointMode(uint8_t servoId);
		bool isMoving(uint8_t servoId);
		
		//---------------------convert----------------------------------
		
		int32_t convertRadian2Value(uint8_t servoId, float radian);
		float convertValue2Radian(uint8_t servoId, int32_t value);

		int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian);
		float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian);

		int32_t convertVelocity2Value(uint8_t servoId, float velocity);
		float convertValue2Velocity(uint8_t servoId, int32_t value);

		int16_t convertCurrent2Value(float current);
		float convertValue2Current(int16_t value);

		float convertValue2Load(int16_t value);
		
		//---------------------------------------------------------------
		bool pingServo (const uint8_t servoId);
		
		bool setServoReturnDelayMicros (const uint8_t servoId, const uint16_t micros);
		bool setServoBlinkConditions (const uint8_t servoId, const uint8_t errorFlags);
		// set the errors that will cause the servo to shut off torque
		bool setServoShutdownConditions (const uint8_t servoId, const uint8_t errorFlags);

		
		bool setServoMaxTorque(const uint8_t servoId, const uint16_t torqueValue);
		/*
		You can use 0 ~ 2,047 (0x7FF) and the unit is 4.5mA (Torque is directly proportional to the current value).
		If you use a value between 0 ~ 1,023, torque is applied to the CCW direction, and setting it to 0 will stops.
		If you use a value between 1,024 ~ 2,047, torque is applied to the CW direction, and setting it to 1,024 will stops.
		That means, 10th bit becomes the CW/CCW direction bit, which controls rotational direction.
		Goal Torque cannot be bigger than Torque Limit(34,35).
		*/
		bool setServoGoalTorque (const uint8_t servoId, const uint16_t torqueValue);

		bool getServoGoalTorque (const uint8_t servoId, uint16_t *torqueValue);
		
		bool getPresentLoad(const uint8_t servoId, uint16_t *loadValue);

		/*
		Join Mode, Multi-Turn mode It is a moving speed to Goal Position.
		0~1023 (0X3FF) can be used, and the unit is about 0.114rpm.
		If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
		If it is 1023, it is about 116.62rpm.
		For example, if it is set to 300, it is about 34.2 rpm.
		However, the rpm will not exceed the No Load Speed.

		Wheel Mode It is a moving speed to Goal direction.
		0~2047 (0X7FF) can be used, and the unit is about 0.114rpm.
		If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
		If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.
		That is, the 10th bit becomes the direction bit to control the direction.
		*/
		bool setServoGoalVelocity (const uint8_t servoId, const uint16_t speedValue);

		bool getServoGoalVelocity (const uint8_t servoId, uint16_t *speedValue);

		bool getServoPresentVelocity (const uint8_t servoId, int16_t *velocityValue);

		// make the servo move to an angle
		// valid angles are between 0 and 300 degrees
		bool setServoGoalPosition (const uint8_t servoId, const float angle); // TODO
		
		bool setServoGoalPosition (const uint8_t servoId, const int32_t angle);

		bool getServoPresentPosition (const uint8_t servoId, float *angle); // TODO
		
		bool getServoPresentPosition (const uint8_t servoId, int32_t *angle);

		
private:
		void sendServoCommand (const uint8_t servoId, const ServoCommand commandByte, \
														const uint8_t numParams, const uint8_t *params);
											 
		bool getServoResponse (void);
		bool getAndCheckResponse (const uint8_t servoId);
											 
		int getServoBytesAvailable (void);

		void sendServoByte(uint8_t byte);
private:
		//enum{MAX_INFO_SIZE=50};
		u32 m_baudrate;
		ServoResponse m_response;
		ServoInstruction m_servoInstruction;
		//char m_info[MAX_INFO_SIZE];
};

#endif
