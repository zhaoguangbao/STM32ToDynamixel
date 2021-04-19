#include "servo.h" 
#include "string.h"


/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// control table
#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24		// PRESENT_POSITION


// response location
#define SERVO_ID_POS 2
#define SERVO_LEN_POS 3
#define SERVO_ERROR_POS 4
#define SERVO_PARAM_POS 5


// helper function
void HelperPrintInfo(bool result, const char* info){
	if(!result){
		user_main_error("Failed to get/set %s!", info);
	}else{
		user_main_info("Succeeded to get/set %s!", info);
	}
}


// public
int Servo::getTemperature(const uint8_t servoId)
{
	const uint8_t params[2] = {present_temperature,
														 present_temperature_sz};
	sendServoCommand(servoId, READ, 2, params);
															
	if (!getAndCheckResponse (servoId))
		return -1;
  uint8_t tempreture=m_response.params[0];
  return tempreture;
}

bool Servo::torque(uint8_t servoId, uint8_t onoff){
	const uint8_t params[torque_enable_sz+1] = {torque_enable,
																							onoff};
	sendServoCommand(servoId, WRITE, torque_enable_sz+1, params);
																							
	if (!getAndCheckResponse (servoId))
		return false;
	return true;
}
bool Servo::torqueOn(uint8_t servoId){
	return torque(servoId, 1);
}
bool Servo::torqueOff(uint8_t servoId){
	return torque(servoId, 0);
}

bool Servo::changeID(uint8_t servoId, uint8_t new_id){
	bool result = false;

  result = torqueOff(servoId);
  if (result == false) 
		return false;
	
	const uint8_t params[dynamixel_ID_sz+1] = {dynamixel_ID,
																							new_id};
	sendServoCommand(servoId, WRITE, dynamixel_ID_sz+1, params);
  result = getAndCheckResponse (servoId);
  if (result == false) 
  {
    user_main_info("[DynamixelWorkbench] Failed to change ID!");
    return false;
  }
  delay_ms(1000);

  user_main_info("[DynamixelWorkbench] Succeeded to change ID!");
  return result;
}
bool Servo::changeBaudrate(uint8_t servoId, uint32_t new_baudrate){
	bool result = false;

  result = torqueOff(servoId);
	
	uint8_t baud2param=34;
	
  if (result == false) 
		return false;

    switch (new_baudrate)
    {
      case 9600:
				baud2param=207;
       break;

      case 19200:
				baud2param=103;
       break;

      case 57600:
				baud2param=34;
       break;

      case 115200:
				baud2param=16;
       break;

      case 200000:
				baud2param=9; 
       break;

      case 250000:
				baud2param=7;    
       break;
       
      case 400000:
				baud2param=4; 
       break;

      case 500000:
				baud2param=3;    
       break;

      case 1000000:
				baud2param=1;    
       break;

      case 2250000:
				baud2param=250;      
       break;

      case 2500000:
				baud2param=251;     
       break;

      case 3000000:
				baud2param=252;    
       break;
       
      default:
				baud2param=34;
       break;
    }
		
	const uint8_t params[dynamixel_baudrate_sz+1] = {dynamixel_baudrate,
																								baud2param};
	sendServoCommand(servoId, WRITE, dynamixel_baudrate_sz+1, params);
																							
	if (!getAndCheckResponse (servoId))
		return false;
	return true;
}
bool Servo::changeProtocolVersion(uint8_t servoId, uint8_t version){
	// TODO
	return true;
}

bool Servo::led(uint8_t servoId, int32_t onoff)
{
	bool result = false;
	
	const uint8_t params[dynamixel_LED_sz+1] = {dynamixel_LED,
																							onoff};
	sendServoCommand(servoId, WRITE, dynamixel_LED_sz+1, params);
																							
	result=getAndCheckResponse(servoId);
																							
  if (result == false)
  {
    user_main_info("[DynamixelWorkbench] Failed to change led status!");
    return false;
  }

  user_main_info("[DynamixelWorkbench] Succeeded to change led status!");
  return result;
}
bool Servo::ledOn(uint8_t servoId)
{
	return led(servoId, 1);
}
bool Servo::ledOff(uint8_t servoId)
{
	return led(servoId, 0);
}



bool Servo::setNormalDirection(uint8_t servoId)
{
	// TODO
	return true;
}
bool Servo::setReverseDirection(uint8_t servoId)
{
	// TODO
	return true;
}
		
bool Servo::setSecondaryID(uint8_t servoId, uint8_t secondary_id)
{
	// TODO
	return true;
}

bool Servo::setOperatingMode(uint8_t servoId, uint8_t index)
{
	bool result=false;
	switch (index){
		case POSITION_CONTROL_MODE:
		{
			uint8_t highByte = DXL_HIBYTE(0);
			uint8_t lowByte = DXL_LOBYTE(0);
			uint8_t cw_params[CW_angle_limit_sz+1]={CW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CW_angle_limit_sz+1, cw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "POSITION_CONTROL_MODE-CW_angle_limit");
			
			highByte = DXL_HIBYTE(4095);
			lowByte = DXL_LOBYTE(4095);
			uint8_t ccw_params[CCW_angle_limit_sz+1]={CCW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CW_angle_limit_sz+1, ccw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "POSITION_CONTROL_MODE-CCW_angle_limit");
			break;
		}
		case VELOCITY_CONTROL_MODE:
		{
			uint8_t highByte = DXL_HIBYTE(0);
			uint8_t lowByte = DXL_LOBYTE(0);
			uint8_t cw_params[CW_angle_limit_sz+1]={CW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CW_angle_limit_sz+1, cw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "VELOCITY_CONTROL_MODE-CW_angle_limit");
			
			highByte = DXL_HIBYTE(0);
			lowByte = DXL_LOBYTE(0);
			uint8_t ccw_params[CCW_angle_limit_sz+1]={CCW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CCW_angle_limit_sz+1, ccw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "VELOCITY_CONTROL_MODE-CCW_angle_limit");
			break;
		}
		case CURRENT_CONTROL_MODE:
		{
			// not support
			break;
		}
		case TORQUE_CONTROL_MODE:
		{
			uint8_t value=1;
			uint8_t params[torque_control_mode_sz+1]={torque_control_mode, value};
			sendServoCommand (servoId, WRITE, torque_control_mode_sz+1, params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "TORQUE_CONTROL_MODE");
			break;
		}
		case MULTI_TURN_MODE:
		{
			uint8_t highByte = DXL_HIBYTE(4095);
			uint8_t lowByte = DXL_LOBYTE(4095);
			uint8_t cw_params[CW_angle_limit_sz+1]={CW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CW_angle_limit_sz+1, cw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "MULTI_TURN_MODE-CW_angle_limit");
			
			highByte = DXL_HIBYTE(4095);
			lowByte = DXL_LOBYTE(4095);
			uint8_t ccw_params[CCW_angle_limit_sz+1]={CCW_angle_limit, lowByte, highByte};
			sendServoCommand (servoId, WRITE, CCW_angle_limit_sz+1, ccw_params);
			result=getAndCheckResponse (servoId);
			
			HelperPrintInfo(result, "MULTI_TURN_MODE-CCW_angle_limit");
			break;
		}
		case CURRENT_BASED_POSITION_CONTROL_MODE:
		{
			// not support
			break;
		}
		case PWM_CONTROL_MODE:
		{
			// not support
			break;
		}
		default:
		{
			user_main_error("Please set suitable operating mode!!");
			return false;
		}
	}
	return true;
}

bool Servo::jointMode(uint8_t servoId, int32_t velocity, int32_t acceleration)
{
	bool result = torqueOff(servoId);
	if (result == false) 
		return false;
	result = setOperatingMode(servoId, POSITION_CONTROL_MODE);
  if (result == false) 
		return false;
	//goal_velocity
	uint8_t highByte = DXL_HIBYTE(velocity);
	uint8_t lowByte = DXL_LOBYTE(velocity);
	uint8_t vel_params[goal_velocity_sz+1]={goal_velocity, lowByte, highByte};
	sendServoCommand (servoId, WRITE, goal_velocity_sz+1, vel_params);
	result=getAndCheckResponse (servoId);
			
	HelperPrintInfo(result, "goal_velocity");
	// goal_acceleration
	uint8_t acc_params[goal_acceleration_sz+1]={goal_acceleration, (uint8_t)acceleration};
	sendServoCommand (servoId, WRITE, goal_acceleration_sz+1, acc_params);
	result=getAndCheckResponse (servoId);
			
	HelperPrintInfo(result, "goal_acceleration");
	
	result = torqueOn(servoId);
  if (result == false) 
		return false;
	
	return true;
}
bool Servo::wheelMode(uint8_t servoId, int32_t acceleration)
{
	bool result = torqueOff(servoId);
	result=setOperatingMode(servoId, VELOCITY_CONTROL_MODE);
	if(result==false)
		return false;
	// goal_acceleration
	uint8_t acc_params[goal_acceleration_sz+1]={goal_acceleration, (uint8_t)acceleration};
	sendServoCommand (servoId, WRITE, goal_acceleration_sz+1, acc_params);
	result=getAndCheckResponse (servoId);
	HelperPrintInfo(result, "goal_acceleration");
	
	result = torqueOn(servoId);
  if (result == false) 
		return false;
	return true;
}

bool Servo::isJointMode(uint8_t servoId)
{
	uint16_t ccw_angle_limit=0;
	const uint8_t params[2] = {CCW_angle_limit,
                             CCW_angle_limit_sz};
	
	sendServoCommand (servoId, READ, 2, params);

	bool result=getAndCheckResponse (servoId);
  if (result == false)
  {
    user_main_error("Failed to judge isJointMode!");
    return result;
  }
	
	ccw_angle_limit=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);

  return ccw_angle_limit; //4095 -- joint mode; 0 -- wheel mode
}

bool Servo::isMoving(uint8_t servoId)
{
	bool res;
	const uint8_t params[2] = {is_moving,
                             is_moving_sz};
	
	sendServoCommand (servoId, READ, 2, params);

	bool result=getAndCheckResponse (servoId);
	if (result == false)
  {
    user_main_error("Failed to judge isJointMode!");
    return result;
  }
	res=m_response.params[0];
	return res;
}


int32_t Servo::convertRadian2Value(uint8_t servoId, float radian)
{
	// TODO
	return true;
}
float Servo::convertValue2Radian(uint8_t servoId, int32_t value)
{
	// TODO
	return true;
}

int32_t Servo::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
	int32_t value = 0;
  int32_t zero_position = (max_position + min_position)/2;

  if (radian > 0)
  {
    value = (radian * (max_position - zero_position) / max_radian) + zero_position;
  }
  else if (radian < 0)
  {
    value = (radian * (min_position - zero_position) / min_radian) + zero_position;
  }
  else
  {
    value = zero_position;
  }

  return value;
}
float Servo::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
	float radian = 0.0;
  int32_t zero_position = (max_position + min_position)/2;

  if (value > zero_position)
  {
    radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
  }
  else if (value < zero_position)
  {
    radian = (float)(value - zero_position) * min_radian / (float)(min_position - zero_position);
  }

  return radian;
}

int32_t Servo::convertVelocity2Value(uint8_t servoId, float velocity)
{
	// TODO
	return true;
}
float Servo::convertValue2Velocity(uint8_t servoId, int32_t value)
{
	// TODO
	return true;
}

int16_t Servo::convertCurrent2Value(float current)
{
	int16_t value = 0;
  const float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  value = current / CURRENT_UNIT;

  return value;
}
float Servo::convertValue2Current(int16_t value)
{
	float current = 0;
  const float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  current = (int16_t)value * CURRENT_UNIT;

  return current;
}

float Servo::convertValue2Load(int16_t value)
{
	float load = 0;
  const float LOAD_UNIT = 0.1f; //Unit : %, Ref : http://emanual.robotis.com/docs/en/dxl/mx/mx-28/#present-load

  if (value == 1023 || value == 0) load = 0.0f;
  else if (value > 0 && value < 1023) load = value * LOAD_UNIT;
  else if (value > 1023 && value < 2048) load = (value - 1023) * LOAD_UNIT * (-1.0f);

  return load;
}


// ping a servo, returns true if we get back the expected values
bool Servo::pingServo (const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::setServoReturnDelayMicros (const uint8_t servoId, const uint16_t micros)
{
    if (micros > 510)
        return false;
    
    const uint8_t params[return_delay_time_sz+1] = {return_delay_time,
                               (uint8_t)((micros / 2) & 0xff)}; // unit 2
    
    sendServoCommand (servoId, WRITE, return_delay_time_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to blink its LED
bool Servo::setServoBlinkConditions (const uint8_t servoId, const uint8_t flags)
{
    const uint8_t params[alarm_LED_sz+1] = {alarm_LED,
                               flags};
    
    sendServoCommand (servoId, WRITE, alarm_LED_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to shut off torque
bool Servo::setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t flags)
{
    const uint8_t params[alarm_shutdown_sz+1] = {alarm_shutdown,
                               flags};
    
    sendServoCommand (servoId, WRITE, alarm_shutdown_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}


bool Servo::setServoMaxTorque(const uint8_t servoId, const uint16_t torqueValue)
{
		const uint8_t highByte = DXL_HIBYTE(torqueValue);
    const uint8_t lowByte = DXL_LOBYTE(torqueValue);
    
    if (torqueValue > 1023)
        return false;
    
    const uint8_t params[max_torque_sz+1] = {max_torque,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, max_torque_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::setServoGoalTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = DXL_HIBYTE(torqueValue);
    const uint8_t lowByte = DXL_LOBYTE(torqueValue);
    
    if (torqueValue > 2047)
        return false;
    
    const uint8_t params[torque_control_goal_sz+1] = {torque_control_goal,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, torque_control_goal_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoGoalTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {torque_control_goal,
                               torque_control_goal_sz};
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
		
		*torqueValue=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);
    
    return true;
}

bool Servo::getPresentLoad(const uint8_t servoId, uint16_t *loadValue)
{
		const uint8_t params[2] = {present_load,
                               present_load_sz};
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
		
		*loadValue=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);
    
    return true;
}

bool Servo::setServoGoalVelocity (const uint8_t servoId,
                       const uint16_t speedValue)
{
    const uint8_t highByte = DXL_HIBYTE(speedValue);
    const uint8_t lowByte = DXL_LOBYTE(speedValue);
	
		bool bJointMode=isJointMode(servoId);
		bool result=true;
		if(bJointMode){
			if (speedValue > 1023)
				result=false;
		}else{
			if(speedValue>2047)
				result=false;
		}
		if(!result){
			user_main_error("Speed is out of range in joint mode(0<speed<=1023)!!");
			return false;
		}
    
    const uint8_t params[goal_velocity_sz+1] = {goal_velocity, lowByte, highByte};
    
    sendServoCommand (servoId, WRITE, goal_velocity_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoGoalVelocity (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {goal_velocity,
                               goal_velocity_sz};  // read two bytes, starting at address MAX_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
		*speedValue=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);
    
    return true;
}

bool Servo::getServoPresentVelocity (const uint8_t servoId,
                              int16_t *velocityValue)
{
    const uint8_t params[2] = {present_velocity,
                               present_velocity_sz};  // read two bytes, starting at address CURRENT_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
		*velocityValue=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);
    
    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool Servo::setServoGoalPosition (const uint8_t servoId,
                    const float angle)
{
	bool result = 0;
  uint32_t value = 0;

  value = convertRadian2Value(servoId, angle);

  result = setServoGoalPosition(servoId, (int32_t)value);
  HelperPrintInfo(result, "goalPosition");
  return true;
}

bool Servo::setServoGoalPosition (const uint8_t servoId,
												const int32_t angle)
{
	if (angle < 0 || angle > 0xfff)
        return false;
    
    const uint8_t highByte = DXL_HIBYTE(angle);
    const uint8_t lowByte = DXL_LOBYTE(angle);
    
    const uint8_t params[goal_position_sz+1] = {goal_position,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, goal_position_sz+1, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool Servo::getServoPresentPosition (const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    uint16_t angleValue = m_response.params[1];
    angleValue <<= 8;
    angleValue |= m_response.params[0];
    
    *angle = (float)angleValue * 300.0 / 1023.0;
    
    return true;
}

bool Servo::getServoPresentPosition (const uint8_t servoId,
												int32_t *angle)
{
	const uint8_t params[2] = {present_position,
                               present_position_sz};  // read two bytes, starting at address CURRENT_ANGLE
    
  sendServoCommand (servoId, READ, 2, params);
    
  if (!getAndCheckResponse (servoId))
      return false;
	
	*angle=DXL_MAKEWORD(m_response.params[0], m_response.params[1]);
	return true;
}



// private
void Servo::sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    sendServoByte (0xff);
    sendServoByte (0xff);  // command header
    
    sendServoByte (servoId);  // servo ID
    uint8_t checksum = servoId;
    
    sendServoByte (numParams + 2);  // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    
    checksum += numParams + 2 + commandByte;
    
    for (uint8_t i = 0; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        checksum += params[i];
				m_servoInstruction.params[i]=params[i];
    }
    
    sendServoByte (~checksum);  // checksum
		
		
		// show command info
		m_servoInstruction.header1=0xff;
		m_servoInstruction.header2=0xff;
		m_servoInstruction.id=servoId;
		m_servoInstruction.length=numParams+2;
		m_servoInstruction.command=(uint8_t)commandByte;
		m_servoInstruction.checksum=~checksum;
		
		RS485_RX_CNT=0; // 接收数据清零
}

bool Servo::getServoResponse (void)
{
    uint8_t retries = 0;
		uint8_t res[REC_BUFFER_LEN];
		uint8_t len;
    
    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
					user_main_error("Too many retries at start");
            return false;
        }
        
        delay_us (REC_WAIT_START_US); // delay_us
    }
    retries = 0;
		
		RS485_Receive_Data(res, &len);
		
		//m_servoInstruction // 在接收到响应后 打印发送指令
		
		user_main_info("%d %d %d %d %d", \
									m_servoInstruction.header1, m_servoInstruction.header2, \
									m_servoInstruction.id, m_servoInstruction.length, m_servoInstruction.command);
		for(int i=0;i<m_servoInstruction.length-2;++i){
			user_main_info("%d", m_servoInstruction.params[i]);
		}
		user_main_info("%d", m_servoInstruction.checksum);

    m_response.id = res[SERVO_ID_POS];
    m_response.length = res[SERVO_LEN_POS];
		
    if (m_response.length > SERVO_MAX_PARAMS)
    {
        user_main_error("Response length too big: %d", (int)m_response.length);
        return false;
    }
    
		
		if(len-SERVO_LEN_POS < m_response.length-1) // -1 or 0
		{
     user_main_error("Too many retries waiting for params, got %d of %d params", getServoBytesAvailable(), m_response.length);
     return false;      
		}
    
    m_response.error = res[SERVO_ERROR_POS];
    
    for (uint8_t i = 0; i < m_response.length - 2; i++)
        m_response.params[i] = res[SERVO_PARAM_POS+i];
		
		user_main_debug("Response %d, %d, %d", (int)m_response.id, (int)m_response.length, (int)m_response.error);
		for (uint8_t i = 0; i < m_response.length - 2; i++)
        user_main_debug("%d", m_response.params[i]);
    
    
    uint8_t calcChecksum = m_response.id + m_response.length + m_response.error;
    for (uint8_t i = 0; i < m_response.length - 2; i++)
        calcChecksum += m_response.params[i];
    calcChecksum = ~calcChecksum;
    
    const uint8_t recChecksum = res[len-1];
    if (calcChecksum != recChecksum)
    {
        user_main_error("Checksum mismatch: %d calculated, %d received", calcChecksum, recChecksum);
        return false;
    }
    
    return true;
}

bool Servo::getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        user_main_error("Servo error: Servo %d did not respond correctly or at all", (int)servoId);
        return false;
    }
    
    if (m_response.id != servoId)
    {
        user_main_error("Servo error: Response ID %d does not match command ID %d", (int)m_response.id, servoId);
        return false;
    }
    
    if (m_response.error != 0)
    {
        user_main_error("Servo error: Response error code was nonzero (%d)", (int)m_response.error);
        return false;
    }
    
    return true;
}


int Servo::getServoBytesAvailable (void)
{
	return RS485_RX_CNT;
}


void Servo::sendServoByte (uint8_t byte)
{
	RS485_Send_Data(&byte, 1);
}
