
#include <Wire.h>
#include <MPU6050_light.h>

#include "../include/turtle_bot.hpp"
#include "../include/WiFiHardware.hpp"

turtle_bot tb;
MPU6050 imu(Wire);
ros::NodeHandle_<WiFiHardware> nh;

ros::Publisher pub_raw_odom_data("/mytb3/raw_odom_data", &tb.raw_odom_data);


IPAddress ros_server(192, 168, 1, 12);
const uint16_t esp_port = 11411; 
const char* ssid = "TEData";
const char* pass = "25172233";


void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/mytb3/cmd_vel", &cmd_vel_cb);

void setup() 
{
	Wire.begin();
	Serial.begin(115200);

	pinMode(BUILTIN_LED, OUTPUT);

    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) 
    {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(100);
        digitalWrite(BUILTIN_LED, LOW);
        delay(100);

        if (millis() > 5000) ESP.restart();
    }
   
    nh.getHardware()->setConnection(ros_server, esp_port);
    nh.initNode();

	nh.advertise(pub_raw_odom_data);

	nh.subscribe(sub_cmd_vel);

    while (not nh.connected())
    {
        nh.spinOnce();

        digitalWrite(BUILTIN_LED, HIGH);
        delay(250);
        digitalWrite(BUILTIN_LED, LOW);
        delay(250);
    }

	digitalWrite(BUILTIN_LED, HIGH);

	imu.begin();
	imu.calcOffsets(true, true);

	attachInterrupt(digitalPinToInterrupt(LEFT_MOT_ENCA), left_encoder_handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_MOT_ENCA), right_encoder_handler, CHANGE);
}


void loop() 
{
	uint32_t current_time_ms = millis();

	if (tb.update_motors(current_time_ms))
	{

		tb.update_enc_odom();
		
		if (tb.update_imu_odom(current_time_ms))
		{
			pub_raw_odom_data.publish(&tb.raw_odom_data);
		}	
	}

	
	nh.spinOnce();

	while (not nh.connected()) nh.spinOnce();
}



void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg) 
{
  	double desired_v = cmd_vel_msg.linear.x;
  	double desired_w = cmd_vel_msg.angular.z;

	double max_feasible_v = ((-TB_MAX_LINEAR_VELOCITY / TB_MAX_ANGULAR_VELOCITY) * desired_w) + TB_MAX_LINEAR_VELOCITY;	

	if (max_feasible_v < desired_v) desired_v = max_feasible_v;

	tb.desired_twist.linear.x  = desired_v;
	tb.desired_twist.angular.z = desired_w;

	tb.update_vel();
}


void left_encoder_handler(void)           
{
    if (digitalRead(LEFT_MOT_ENCA) == digitalRead(LEFT_MOT_ENCB)) tb.left_wheel.curr_enc_count--;       // CCW
    else tb.left_wheel.curr_enc_count++;       // CW

    // if (digitalRead(LEFT_MOT_ENCA) == digitalRead(LEFT_MOT_ENCB)) tb.left_wheel.curr_enc_count++;       // CCW
    // else tb.left_wheel.curr_enc_count--;       // CW
}


void right_encoder_handler(void)          
{
    if (digitalRead(RIGHT_MOT_ENCA) == digitalRead(RIGHT_MOT_ENCB)) tb.right_wheel.curr_enc_count++;       // CCW
    else tb.right_wheel.curr_enc_count--;       // CW
	
    // if (digitalRead(RIGHT_MOT_ENCA) == digitalRead(RIGHT_MOT_ENCB)) tb.right_wheel.curr_enc_count--;       // CCW
    // else tb.right_wheel.curr_enc_count++;       // CW
}