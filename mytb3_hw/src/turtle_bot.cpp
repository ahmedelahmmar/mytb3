#include "../include/turtle_bot.hpp"

#include <MPU6050_light.h>

extern turtle_bot tb;
extern MPU6050    imu;

void left_encoder_handler(void);
void right_encoder_handler(void);


turtle_bot::turtle_bot(void):
left_wheel(LEFT_MOT_ENA, LEFT_MOT_IN2, LEFT_MOT_IN1, LEFT_MOT_ENA, LEFT_MOT_ENCB),      // IN1 and IN2 switched intentionally
right_wheel(RIGHT_MOT_ENA, RIGHT_MOT_IN1, RIGHT_MOT_IN2, RIGHT_MOT_ENA, RIGHT_MOT_ENCB)
{
    this->pose.x = this->pose.y = this->pose.theta = 0;
}


bool turtle_bot::update_motors(uint32_t current_time_ms)
{
    return (this->left_wheel.update(current_time_ms) and this->right_wheel.update(current_time_ms));
}


bool turtle_bot::update_imu_odom(uint32_t current_time_ms)
{
    static uint32_t last_call_ms = 0;
    if ((current_time_ms - last_call_ms) > IMU_UPDATE_INTERVAL_MS)
    {
        last_call_ms = current_time_ms;

        imu.update();

        this->raw_odom_data.orientation.x = imu.getAccX() * 9.81;
        this->raw_odom_data.orientation.y = imu.getAccY() * 9.81;
        this->raw_odom_data.orientation.z = radians(imu.getAngleZ());
        this->raw_odom_data.orientation.w = radians(imu.getGyroZ());

        return 1;
    }   
    return 0;
}


void turtle_bot::update_enc_odom(void)
{
    noInterrupts();

    double dx_left  = (double(this->left_wheel.curr_enc_count - this->left_wheel.last_enc_count) / ENCODER_PPR) * 2.0 * PI * WHEEL_RADIUS_METER;
    double dx_right = (double(this->right_wheel.curr_enc_count - this->right_wheel.last_enc_count) / ENCODER_PPR) * 2.0 * PI * WHEEL_RADIUS_METER;

    interrupts();

    double dx_tb    = ((dx_right + dx_left) / 2.0);
    double dtheta   = ((dx_right - dx_left) / WHEEL_BASE);

    this->raw_odom_data.position.x += dx_tb * cos(raw_odom_data.position.z);
    this->raw_odom_data.position.y += dx_tb * sin(raw_odom_data.position.z);

    this->raw_odom_data.position.z += dtheta;

    if (this->raw_odom_data.position.z < -PI) this->raw_odom_data.position.z += (2.0 * PI);
    else if (PI < this->raw_odom_data.position.z) this->raw_odom_data.position.z -= (2.0 * PI);
}


void turtle_bot::update_vel(void)
{
    double v_left_wheel  = this->desired_twist.linear.x - ((this->desired_twist.angular.z * WHEEL_BASE) / 2);
    double v_right_wheel = this->desired_twist.linear.x + ((this->desired_twist.angular.z * WHEEL_BASE) / 2);

    this->left_wheel.setDesiredRpm( ((60 * v_left_wheel) / (2 * PI * WHEEL_RADIUS_METER)) );
    this->right_wheel.setDesiredRpm( ((60 * v_right_wheel) / (2 * PI * WHEEL_RADIUS_METER)) );
}

