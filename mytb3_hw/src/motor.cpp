
#include "../include/motor.hpp"

motor::motor(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t enc_a, uint8_t enc_b):
ena_pin(ena), in1_pin(in1), in2_pin(in2), enc_a_pin(enc_a), enc_b_pin(enc_b), direction(FORWARD), controller(5, 0.95, 0.75)
{
    controller.setOutputLimits(0, 255);
    controller.setIntegralLimits(-25, 25);
    controller.setDerivativeFilterCoeff(0.25);

    pinMode(ena_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    pinMode(enc_a_pin, INPUT_PULLUP);
    pinMode(enc_b_pin, INPUT_PULLUP);

    setDirection(FORWARD);
    stop();
}


bool motor::update(uint32_t current_time_ms)
{
    if ((current_time_ms - this->last_call_ms) > MOTOR_UPDATE_INTERVAL_MS)
    {
        this->last_call_ms = current_time_ms;

        noInterrupts();

        this->rpm = ((this->curr_enc_count * 60000) / (ENCODER_PPR * MOTOR_UPDATE_INTERVAL_MS));

        this->last_enc_count = this->curr_enc_count;
        this->curr_enc_count = 0;
        
        interrupts();

        // analogWrite(this->ena_pin, 255);
        analogWrite(this->ena_pin, this->controller.compute(this->desired_rpm, this->rpm, current_time_ms));
        return 1;
    }   

    return 0;
}


void motor::setDesiredRpm(float desired_rpm)
{
    if (desired_rpm < 0)
    {
        this->setDirection(REVERSE);
        desired_rpm *= -1;
    } 
    else this->setDirection(FORWARD);

    if (MOTOR_MAX_RPM < desired_rpm) desired_rpm = MOTOR_MAX_RPM;
    else if (desired_rpm < MOTOR_MIN_RPM) 
    {
        this->stop();
        return;
    }

    this->desired_rpm = uint8_t(desired_rpm);
}


void motor::setDirection(bool direction)
{
    this->direction = direction;

    switch (this->direction)
    {
        case FORWARD:
            digitalWrite(this->in1_pin, HIGH);
            digitalWrite(this->in2_pin, LOW);
            break;
        
        case REVERSE:
            digitalWrite(this->in1_pin, LOW);
            digitalWrite(this->in2_pin, HIGH);
            break;

        default: break;
    }
}


void motor::stop(void)
{
    digitalWrite(this->in1_pin, LOW);
    digitalWrite(this->in2_pin, LOW);

    this->desired_rpm = 0;
    this->controller.reset();
}