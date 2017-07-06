#include "mavlink.h"
#include "freertos.h"
#include "serial.h"
#include "stm32f7xx_hal.h"
#include "sensor.h"
#define BUFFER_LENGTH 1000

typedef struct
{
  float Roll;
  float Pitch;
  float Yaw;
} Euler_t;


uint8_t tx_buf[BUFFER_LENGTH];  //用于存放发送的原始数据
Euler_t GYR_Value_Rad;
Euler_t Euler_Value_Rad;
mavlink_system_t mavlink_system;
mavlink_message_t msg;

// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_QUADROTOR;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

void mavlink_init(void)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	Euler_Value_Rad.Roll=1.57;
        Euler_Value_Rad.Pitch=1.57;
        Euler_Value_Rad.Yaw=1.57;
        GYR_Value_Rad.Roll=1.57;
        GYR_Value_Rad.Pitch=1.57;
        GYR_Value_Rad.Yaw=1.57;       
}

void Mavlink_Task (void const * argument){
  mavlink_init();
  unsigned short len;
  for(;;){
  vTaskDelay(10);	
      //send heart beat
        mavlink_system.compid = MAV_COMP_ID_ALL;
        mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type,autopilot_type, system_mode, custom_mode, system_state);			
	len = mavlink_msg_to_send_buffer(tx_buf, &msg);
        vSerialPutString(0, ( signed char * )tx_buf,len);         

      //send IMU     
        mavlink_msg_attitude_pack(mavlink_system.sysid,mavlink_system.compid,  &msg,  HAL_GetTick(), 
                                  Euler_Value_Rad.Roll, Euler_Value_Rad.Pitch, Euler_Value_Rad.Yaw, 
                                  GYR_Value_Rad.Roll,GYR_Value_Rad.Pitch, GYR_Value_Rad.Yaw);
        len = mavlink_msg_to_send_buffer(tx_buf, &msg);
        vSerialPutString(0, ( signed char * )tx_buf,len);
  }
}