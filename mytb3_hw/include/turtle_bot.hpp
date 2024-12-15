#ifndef __TURTLE_BOT_H__
#define __TURTLE_BOT_H__


#include "motor.hpp"


class turtle_bot
{
    private:
        geometry_msgs::Twist  desired_twist;

    public:
        geometry_msgs::Pose2D pose;
        geometry_msgs::Pose   raw_odom_data;  
        /*
         geometry_msgs/Point position
             float64 x: calculated x position due to encoder readings (m)
             float64 y: calculated y position due to encoder readings (m)
             float64 z: calculated theta due to encoder readings (radians)
         geometry_msgs/Quaternion orientation
             float64 x: raw imu acceleration component on x axis (m/s2)
             float64 y: raw imu acceleration component on y axis (m/s2)
             float64 w: raw imu angular velocity about z axis (radians/s)
             float64 z: raw imu theta about z axis (radians)
        */

        motor left_wheel;
        motor right_wheel;

        turtle_bot(void);
        bool update_motors(uint32_t current_time_ms);
        bool update_imu_odom(uint32_t current_time_ms);
        void update_enc_odom(void);
        void update_vel(void);

        friend void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg);
};


#endif /* __TURTLE_BOT_H__ */