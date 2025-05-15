#ifndef MOTOR_DRIVE_WITH_FILTERED_PID
#define MOTOR_DRIVE_WITH_FILTERED_PID

/*  Set up the PWM timers as per instructions, prescaler = 3, period = 999 -> f = 21000Hz */

#include "main.h"
#include "MPU9250-DMP.h"

#define CH1 TIM_CHANNEL_1
#define CH2 TIM_CHANNEL_2
#define CH3 TIM_CHANNEL_3
#define CH4 TIM_CHANNEL_4
   
extern uint32_t dutyCycle; //PWm duty cycle

float getYawAngle(); // set MPU_9250 as per instructions and get the results through this function

class MOTOR {
private: 
	uint16_t IN_1;
	uint16_t IN_2;
	int32_t m_speed; // m_speed is a value ranging from -1000 to 1000
	TIM_HandleTypeDef* M_TIM1; // 
	TIM_HandleTypeDef* M_TIM2;
	uint32_t m_channel1;
	uint32_t m_channel2;
public:
	MOTOR(uint16_t PIN_1, uint16_t PIN_2, TIM_HandleTypeDef* F_TIM1, TIM_HandleTypeDef* F_TIM2, uint32_t CHANNEL1, uint32_t CHANNEL2)
	    : IN_1(PIN_1), IN_2(PIN_2), M_TIM1(F_TIM1), M_TIM2(F_TIM2), m_channel1(CHANNEL1), m_channel2(CHANNEL2) {}

	void setSpeed(int32_t speed) { m_speed = speed; };
	void setMotor();
	void stop() { setSpeed(0); setMotor();};
	void Innit(); 
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float dt, float filter_tau)
        : Kp(kp), Ki(ki), Kd(kd), dt(dt), tau(filter_tau),
        integrator(0), prev_error(0), prev_derivative(0), output(0) {
    }
    float update(float setpoint, float measurement);
    void reset();

private:
    float Kp, Ki, Kd;
    float dt;         // Sample time
    float tau;        // Filter time constant
    float integrator;
    float prev_error;
    float prev_derivative;
    float output;

    // First-order low-pass filter on derivative term
    float lowPassFilter(float new_derivative) {
        float alpha = tau / (tau + dt);
        prev_derivative = alpha * prev_derivative + (1 - alpha) * new_derivative;
        return prev_derivative;
    }
};

class Movements {
public:
    Movements(MOTOR* left, MOTOR* right)
        : leftMotor(left), rightMotor(right) {}

    void goStraight(PIDController* leftPID, PIDController* rightPID,
                    float setpoint, float leftMeas, float rightMeas); // both motors forward
    void rotateLeft(PIDController* leftPID, PIDController* rightPID, float setpoint, float leftMeas, float rightMeas);
    bool rotateLeft90Deg(PIDController* leftPID, PIDController* rightPID, float setpoint, float leftMeas, float rightMeas);
    void rotateRight(PIDController* leftPID, PIDController* rightPID, float setpoint, float leftMeas, float rightMeas);
    bool rotateRight90Deg(PIDController* leftPID, PIDController* rightPID, float setpoint, float leftMeas, float rightMeas);
    void stop();                        // stop both motors

private:
    MOTOR* leftMotor;
    MOTOR* rightMotor;
};



class Encoder{
private:
    int32_t LeftPosi, RightPosi;
    int32_t currLeftPosi, currRightPosi, prevLeftPosi, prevRightPosi ; 
    TIM_HandleTypeDef* M_TIM1;
	TIM_HandleTypeDef* M_TIM2;
	uint32_t m_channel1;
	uint32_t m_channel2;
    uint32_t prevLeftTime = 0;
    uint32_t prevRightTime = 0;
    uint32_t pulse;
public:
    Encoder(TIM_HandleTypeDef* F_TIM1, TIM_HandleTypeDef* F_TIM2, uint32_t CHANNEL1, uint32_t CHANNEL2)
        : currLeftPosi(0), currRightPosi(0), prevLeftPosi(0), prevRightPosi(0),
          LeftPosi(0), RightPosi(0),
          M_TIM1(F_TIM1), M_TIM2(F_TIM2),
          m_channel1(CHANNEL1), m_channel2(CHANNEL2),
          prevLeftTime(0), prevRightTime(0), pulse(1) // default pulse = 1 to avoid divide-by-zero
    {}

    void Innit();
    void setPulse(uint32_t E_Pulse) { pulse = E_Pulse; }
    int32_t readLeftEncoderPosi();
    int32_t readRightEncoderPosi();
    int32_t readLeftEncoderSpeed();
    int32_t readRightEncoderSpeed();
};

class IRSensors{
private:
    ADC_HandleTypeDef *m_hadc; //setup adc as continuos conversion 
    uint32_t IRVal;
    uint32_t WallVal; 
public:
    IRSensors(ADC_HandleTypeDef *hadc) : m_hadc(hadc) {}
    void Innit(){HAL_ADC_Start(m_hadc);};
    uint32_t IRCalip(){
        WallVal = HAL_ADC_GetValue(m_hadc);
        return WallVal;
    };
    bool readIR(){
        IRVal = HAL_ADC_GetValue(m_hadc);
        if(IRVal > WallVal ) return 1;
        else return 0;
    };
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); //communication with the ESP 32 Dev Module via Uart 

uint8_t All_IR_Val(IRSensors* IRMostLeft,IRSensors* IRMiddleLeft, IRSensors* IRMiddleRight, IRSensors* IRMostRight);

void MCM_Wall_Following_In_A_StraightLine(
    uint8_t All_IR_VAL,
    Movements* Ctrl,
    PIDController* leftPID,
    PIDController* rightPID,
    float setpoint,
    float leftMeas,
    float rightMeas);

#endif
