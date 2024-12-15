#ifndef PID_HPP
#define PID_HPP

#include <cmath>

class pid 
{
    private:
        // PID parameters
        float kp;  // Proportional gain
        float ki;  // Integral gain
        float kd;  // Derivative gain

        // State variables
        float prevError;    // Error in the previous iteration
        float integral;     // Cumulative integral term
        float outputMin;    // Minimum output limit
        float outputMax;    // Maximum output limit
        float integralMin;  // Minimum integral limit (clamping)
        float integralMax;  // Maximum integral limit (clamping)
        float derivativeFilterCoeff;
        float filteredDerivative;

        uint32_t prevTime;  // Previous time in milliseconds

    public:
        // Constructor
        pid(float p, float i, float d);

        // Compute the PID output
        float compute(float setpoint, float measured, uint32_t currentTime);

        // Set output limits
        void setOutputLimits(float min, float max);

        // Set integral limits
        void setIntegralLimits(float min, float max);

        // Set derivative filter coefficient
        void setDerivativeFilterCoeff(float coeff);

        // Reset internal PID state
        void reset();
};

#endif // PID_HPP
