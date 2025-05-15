#include "main.h"
#include "MOTORDRiVE.h"
#include "MPU9250-DMP.h"
#include <cstdlib> // for abs()
#include <algorithm>
#include <cstdio>
#include <cstring>

extern UART_HandleTypeDef huart1;
extern Encoder encoder; // Make sure the encoder object is defined in your main file and declared as extern here

float getYawAngle(){
    /*MPU9250_DMP(); // Initialize scale constants
  if (MPU9250_begin() != INV_SUCCESS)
     {
         printf("MPU init failed!\n");
         while (1);
     }

     // Set orientation matrix for DMP (identity matrix here, customize if needed)
     const signed char orientationMatrix[9] = {
         1, 0, 0,
         0, 1, 0,
         0, 0, 1
     };
     MPU9250_dmpSetOrientation(orientationMatrix);

     // Enable DMP features
     MPU9250_dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10); // 10 Hz update */
    if (MPU9250_dmpUpdateFifo() == INV_SUCCESS)
	          {
	              MPU9250_computeEulerAngles(true); // Calculate angles in degrees
	              
                  return yaw_inside;
	          }

	          HAL_Delay(100); // 10 Hz
}

void MOTOR::setMotor() {
	if (m_speed > 0) {
		dutyCycle = m_speed * 21;
		__HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, dutyCycle);
		__HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, 0);
	}
	else if(m_speed < 0) {
		dutyCycle = abs(m_speed) * 21;
		__HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, 0);
		__HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, dutyCycle);
	}
	else {
		__HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, 0);
		__HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, 0);
	}
}

void MOTOR::Innit() {
    // Start PWM on both timers
    HAL_TIM_PWM_Start(M_TIM1, m_channel1);
    HAL_TIM_PWM_Start(M_TIM2, m_channel2);

    // Optional: initialize motor with stopped state
    HAL_TIM_PWM_Stop(M_TIM1, m_channel1);
    HAL_TIM_PWM_Stop(M_TIM2, m_channel2);
}

void Movements::goStraight(PIDController* leftPID, PIDController* rightPID,
                           float setpoint, float leftMeas, float rightMeas) {
    float leftOutput = leftPID->update(setpoint, leftMeas);
    float rightOutput = rightPID->update(setpoint, rightMeas);

    // Clamp outputs to safe range (e.g., -1000 to +1000)
    if (leftOutput > 1000) leftOutput = 1000;
    if (leftOutput < -1000) leftOutput = -1000;
    if (rightOutput > 1000) rightOutput = 1000;
    if (rightOutput < -1000) rightOutput = -1000;

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}


void Movements::rotateLeft(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas) {

    float leftOutput = leftPID->update(-setpoint, leftMeas);   // Reverse left motor
    float rightOutput = rightPID->update(setpoint, rightMeas); // Forward right motor

    if (leftOutput > 1000) leftOutput = 1000;
    if (leftOutput < -1000) leftOutput = -1000;
    if (rightOutput > 1000) rightOutput = 1000;
    if (rightOutput < -1000) rightOutput = -1000;

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}

void Movements::rotateRight(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas) {

    float leftOutput = leftPID->update(setpoint, leftMeas);    // Forward left motor
    float rightOutput = rightPID->update(-setpoint, rightMeas); // Reverse right motor

    if (leftOutput > 1000) leftOutput = 1000;
    if (leftOutput < -1000) leftOutput = -1000;
    if (rightOutput > 1000) rightOutput = 1000;
    if (rightOutput < -1000) rightOutput = -1000;

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}

bool Movements::rotateLeft90Deg(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas){ // has to be called in the while loop
    static bool initialized = false;
    static float targetYaw = 0.0f;

    if (!initialized)
    {
        float currentYaw = getYawAngle(); // from MPU9250 global
        targetYaw = currentYaw - 90.0f;
        if (targetYaw < 0) targetYaw += 360.0f;
        initialized = true;
    }

    float error = yaw_inside - targetYaw;
    if (error < 0) error += 360.0f;

    if (error > 2.0f) // Continue rotating until within 2°
    {
        rotateLeft(leftPID, rightPID, setpoint, leftMeas, rightMeas);
        return 0;
    }
    else
    {
        leftMotor->stop();
        rightMotor->stop();
        initialized = false; // reset for next time
        return 1;
    } 
}

bool Movements::rotateRight90Deg(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas){ // has to be called in the while loop 
    static bool initialized = false;
    static float targetYaw = 0.0f;

    if (!initialized)
    {
        float currentYaw = getYawAngle();
        targetYaw = currentYaw + 90.0f;
        if (targetYaw >= 360.0f) targetYaw -= 360.0f;
        initialized = true;
    }
    
    float error = yaw_inside - targetYaw;
    if (error < 0) error += 360.0f;

    if (error > 2.0f) // Continue rotating until within 2°
    {
        rotateRight(leftPID, rightPID, setpoint, leftMeas, rightMeas);
        return 0;
    }
    else
    {
        leftMotor->stop();
        rightMotor->stop();
        initialized = false; // reset for next time
        return 1;
    } 
    }

void Movements::stop() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
    leftMotor->setMotor();
    rightMotor->setMotor();
}


float PIDController :: update(float setpoint, float measurement) {
    float error = setpoint - measurement;
    // Proportional
    float P = Kp * error;

    // Integral
    integrator += error * dt;
    float I = Ki * integrator;

    // Derivative (with low-pass filter)
    float derivative = (error - prev_error) / dt;
    float D = Kd * lowPassFilter(derivative);
    prev_error = error;

    output = P + I + D;
    return output;
}

void PIDController :: reset() {
    integrator = 0;
    prev_error = 0;
    prev_derivative = 0;
    output = 0;
}

void Encoder :: Innit(){
    HAL_TIM_Encoder_Start(M_TIM1, m_channel1);
    HAL_TIM_Encoder_Start(M_TIM2, m_channel2);

    prevLeftPosi = __HAL_TIM_GET_COUNTER(M_TIM1);
    prevRightPosi = __HAL_TIM_GET_COUNTER(M_TIM2);
}

int32_t Encoder :: readLeftEncoderPosi(){
    currLeftPosi = __HAL_TIM_GET_COUNTER(M_TIM1);
    int16_t posiIncrease = (int16_t)(currLeftPosi - prevLeftPosi); //deals with over and underflow
    LeftPosi += posiIncrease; 
    prevLeftPosi = currLeftPosi;
    return LeftPosi;
}

int32_t Encoder :: readRightEncoderPosi(){
    currRightPosi = __HAL_TIM_GET_COUNTER(M_TIM2);
    int16_t posiIncrease = (int16_t)(currRightPosi - prevRightPosi);
    RightPosi += posiIncrease; 
    prevRightPosi = currRightPosi;
    return RightPosi;
}

int32_t Encoder::readLeftEncoderSpeed() {
    uint32_t now = HAL_GetTick();  // time in milliseconds
    currLeftPosi = __HAL_TIM_GET_COUNTER(M_TIM1);

    int16_t deltaPos = (int16_t)(currLeftPosi - prevLeftPosi);
    uint32_t deltaTime = now - prevLeftTime;

    prevLeftPosi = currLeftPosi;
    prevLeftTime = now;

    if (deltaTime == 0) return 0; // avoid division by zero

    float speed = (float)deltaPos / (float)deltaTime; // counts per ms
    speed *= pulse; // convert to counts per second

    return (int32_t)speed;
}

int32_t Encoder::readRightEncoderSpeed() {
    uint32_t now = HAL_GetTick();
    currRightPosi = __HAL_TIM_GET_COUNTER(M_TIM2);

    int16_t deltaPos = (int16_t)(currRightPosi - prevRightPosi);
    uint32_t deltaTime = now - prevRightTime;

    prevRightPosi = currRightPosi;
    prevRightTime = now;

    if (deltaTime == 0) return 0;

    float speed = (float)deltaPos / (float)deltaTime; // counts per ms
    speed *= pulse;

    return (int32_t)speed;   
}

uint8_t All_IR_Val(IRSensors* IRMostLeft, IRSensors* IRMiddleLeft, IRSensors* IRMiddleRight, IRSensors* IRMostRight) {
    uint8_t all_ir = 0;

    // Left to right: ML | MLft | MRgt | MR
    all_ir |= (IRMostLeft->readIR()     << 3); // Bit 3
    all_ir |= (IRMiddleLeft->readIR()   << 2); // Bit 2
    all_ir |= (IRMiddleRight->readIR()  << 1); // Bit 1
    all_ir |= (IRMostRight->readIR()    << 0); // Bit 0

    return all_ir; 
}

void MCM_Wall_Following_In_A_StraightLine(
    uint8_t All_IR_VAL,
    Movements* Ctrl,
    PIDController* leftPID,
    PIDController* rightPID,
    float setpoint,
    float leftMeas,
    float rightMeas)
{
	switch (All_IR_VAL)
    {
    case 0b0110:
        /* code */
        Ctrl->goStraight(leftPID, rightPID,setpoint,leftMeas,rightMeas);
        break;

    case 0b1110: //titlting a bit to the left 
        Ctrl->rotateRight(leftPID, rightPID,setpoint,leftMeas,rightMeas);
        break;

    case 0b1100: //titlting a bit too much to the left 
        Ctrl->rotateRight(leftPID,rightPID,2*setpoint,leftMeas,rightMeas);
        break;
        
    case 0b0111: //titlting a bit to the right 
        Ctrl->rotateLeft(leftPID, rightPID,setpoint,leftMeas,rightMeas);
        break;

    case 0b0011: //titlting a bit too much to the right 
        Ctrl->rotateLeft(leftPID, rightPID,2*setpoint,leftMeas,rightMeas);
        break;

    case 0b1111: //wall is infront of you 
        Ctrl->stop();
        break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	if(GPIO_Pin == GPIO_PIN_1){
		int32_t leftSpeed = encoder.readLeftEncoderSpeed();
		int32_t rightSpeed = encoder.readRightEncoderSpeed();
        char tx_mes[64];
		snprintf(tx_mes, sizeof(tx_mes), "L:%ld R:%ld\r\n", leftSpeed, rightSpeed);
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_mes, strlen(tx_mes), HAL_MAX_DELAY);	}
}


