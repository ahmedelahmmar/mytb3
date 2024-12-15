#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "../lib/prj.hpp"
#include "../include/pid.hpp"


class motor
{
    private:
        const uint8_t ena_pin;
        const uint8_t in1_pin;
        const uint8_t in2_pin;
        const uint8_t enc_a_pin;
        const uint8_t enc_b_pin;

        uint32_t last_call_ms{0};
        uint8_t desired_rpm{0};
        bool direction{FORWARD};
        pid controller;

    public:
        volatile int32_t curr_enc_count{0};
        volatile int32_t last_enc_count{0};
        
        float rpm;
        float vel;

        motor(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t enc_a, uint8_t enc_b);
        bool update(uint32_t current_time_ms);
        void setDesiredRpm(float desired_rpm);
        void setDirection(bool direction);
        void stop(void);

        friend void left_encoder_handler(void);
        friend void right_encoder_handler(void);
};  

void left_encoder_handler(void);   
void right_encoder_handler(void);   

#endif /* __MOTOR_H__ */