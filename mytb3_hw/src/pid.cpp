#include "../include/pid.hpp"

// Constructor
pid::pid(float p, float i, float d) :kp(p), ki(i), kd(d), prevError(0), prevTime(0), integral(0), filteredDerivative(0) {}

// Compute the PID output
float pid::compute(float setpoint, float measured, uint32_t currentTime) {

    // Calculate error
    float error = setpoint - std::abs(measured);

    // Calculate time difference
    float dt = (currentTime - prevTime) / 1000.0; // Convert ms to seconds

    // Proportional term
    float proportional = kp * error;

    // Derivative term with noise filtering
    float rawDerivative = 0;
    if (dt > 0) rawDerivative = (error - prevError) / dt;

    // Low-pass filter for the derivative term
    filteredDerivative = (1 - derivativeFilterCoeff) * filteredDerivative + derivativeFilterCoeff * rawDerivative;
    float derivativeTerm = kd * filteredDerivative;

    // Include the integral term in the preliminary output
    float integralTerm = ki * integral;

    // Compute preliminary output
    float preliminaryOutput = proportional + integralTerm + derivativeTerm;

    // Determine if output is saturating
    bool isSaturating = ((preliminaryOutput > outputMax) or (preliminaryOutput < outputMin));

    // Determine if error and controller output have the same sign
    bool sameSign = ((error > 0) and (preliminaryOutput > 0)) or ((error < 0) and (preliminaryOutput < 0));

    // Anti-windup clamping: Update integral only if not saturating or error signs differ
    if (not (isSaturating and sameSign)) integral += error * dt;

    // Update integral term after clamping
    integralTerm = ki * integral;

    // Compute final PID output
    float output = proportional + integralTerm + derivativeTerm;

    // Clamp output to limits
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    // Save error and time for next iteration
    prevError = error;
    prevTime = currentTime;

    return output;
}

// Reset the PID state
void pid::reset() {
    prevError = 0;
    integral = 0;
    filteredDerivative = 0; // Reset the filtered derivative
}

// Set output limits
void pid::setOutputLimits(float min, float max) 
{
    outputMin = min;
    outputMax = max;
}

// Set integral limits
void pid::setIntegralLimits(float min, float max) 
{
    integralMin = min;
    integralMax = max;
}

// Set derivative filter coefficient
void pid::setDerivativeFilterCoeff(float coeff) 
{
    derivativeFilterCoeff = coeff;
}