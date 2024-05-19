# Fish Chassis System Protocol
*深圳科创学院2024高中生机器人冬令营 底盘通讯系统*   
*InnoxSZ 2024 Robotics Winter Camp of High School Students.* 

This is the protocol of Fish Chassis System(CHS). Based on Mavlink v2.     
Author: qianwan.Jin

>Version: 2.2   
Generate Date: 2024/01/31     
Description: Fix ducuments  

>Version: 2.1   
Generate Date: 2024/01/25     
Description: Add IMU message  

>Version: 2.0   
Generate Date: 2024/01/24     
Description: Add remoter message  

>Version: 1.0   
Generate Date: 2023/12/27     
Description: Creat this protocol  
Source: [FishChassis.xml](./FishChassis.xml)

- [Fish Chassis System Protocol](#fish-chassis-system-protocol)
  - [Enum](#enum)
  - [Message](#message)
      - [Overall](#overall)
      - [Elaborate](#elaborate)
  - [Sample](#sample)
      - [Logic](#logic)
      - [Sample C code](#sample-c-code)

## Enum
- **CHS_SYSTEM_ID**   
  System ID indicates source of message source.
  > While using mavlink transmission functions, both system_id and component_id should type in a same **CHS_SYSTEM_ID** value, which indicate the sender.

  | Name                            | ID  | Description        |
  | ------------------------------- | --- | ------------------ |
  | CHS_ID_ORANGE         | 0   |  OrangePi.    |
  | CHS_ID_ESP32 | 1   | ESP32 controller. |
  | CHS_ID_CHASSIS | 2   | Chassis. |


## Message
#### Overall
| Name                | ID | Sender     | Receiver   | Trigger Condition                                                     | Description                     |
| ------------------- | ---------- | ---------- | ---------- | --------------------------------------------------------------------- | ------------------------------- |
| chs_ctrl_info     | 0 | OrangePi | Chassis | Actively send | Control chassis' move.       |
| chs_motor_info  | 1 | OrangePi or ESP32     | Chassis | Actively send | Control motors directly. |
| chs_odom_info  | 2 | Chassis | OrangePi or ESP32 | Periodic 200Hz | Feedback variables of chassis. |
| chs_imu_info | 3 | Chassis | OrangePi | Actively send | Feedback accelmeter and gyrometer information. |
| chs_servos_info | 4 | OrangePi or ESP32 | Chassis | Actively send | Control servo sockets pwm duty cycle. |
| chs_manage_info | 5 | OrangePi or ESP32 | Chassis | Actively send | Control motors and servos enable or not, and reset quaterinon. |
| chs_remoter_info | 6 | Chassis | OrangePi or ESP32 | Actively send | Feedback remoter information. |

><font color=red>Fatal Warning: the Motor and the Servo components will auto disable if it doesn't recevie any of valid control message in 500ms.</font>

><font color=orange>Warning: When controlling the Motor and the Servo components, the Chassis will received the earliest control message source and ignore others. For example, the chassis would ignore control messages from OrangePi until it no longer receives the latest control message from ESP32 within 500 ms. When usng a new message source, remember to enable chassis first.</font>

#### Elaborate
- **chs_ctrl_info**   
  Upper control chassis velocity. Coordinate right-forward-up.

  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | vx | float |   m/s   | Indicate velocity of x direction. Range [-2.0, 2.0]. |
  | vy | float |   m/s   | Indicate velocity of x direction. Range [-2.0, 2.0]. |
  | vw | float |   rad/s   | Indicate angular velocity of w direction. Range [-2\*PI, 2\*PI]. |


- **chs_motor_info**   
  Low layer API for control chassis velocity. Coordinate right-forward-up.

  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | motor | int16_t[4] | rpm | Indicate angular speed of wheel motors. Range [-6000, 6000]. |

- **chs_odom_info**      
  Feedback of chassis variables.
  
  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | vx         | float | m/s | Feedback the velocity of x direction.|
  | vy      | float  | m/s | IFeedback the velocity of y direction.|
  | vw | float | rad/s | Feedback the angular velocity of z yaw.(Same to gyro[2]) |
  | quaternion      | float[4] | 1 | Indicate the quaternion. |

- **chs_imu_info**      
  Feedback accelmeter and gyrometer information.
  
  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | accel      | float[3] | m/s^2 | Feedback accelmeter message.|
  | gyro       | float[3] | rad/s | Feedback gyrometer message.|


- **chs_servos_info**
  Control servo pwm generating.  

  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | servos  | uint16_t[7]  | 500us | Indicate duty cycle of servo timer. Range from 499 to 2499 indicates 500/20000 to 2500/20000, frequency is 50Hz. |


- **chs_manage_info**
  Control chassis items.

  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | enable_chassis  | uint8_t  | / | Boolean. Set 1/0 to enable/disable chassis's close-loop control. Invalid with UINT8_MAX. |
  | enable_servos  | uint8_t  | / | Boolean. Set true to enable/disable servo pwm generating. Invalid with UINT8_MAX. |
  | reset_quaternion | uint8_t | / | Boolean. Set true to reset quaternion. Invalid with UINT8_MAX. |

- **chs_remoter_info**
Feedback remoter message.

  | Field      | Type     | Units  | Description |
  | ---------- | -------- | :----: | ----------- |
  | switch_messgae  | uint8_t  | / | Bit-0:isOnline(0-offline,1-online), Bit[1-2]:SwitchLeft(0-0b00,1-0b01,2-0x10), Bit[3-4]:SwitchRight(0-0b00,1-0b01,2-0x10) |
  | channel_0  | int16_t  | / | -660 to +660. Right-Upward is orientation. |
  | channel_1 | int16_t | / | -660 to +660. Right-Upward is orientation. |
  | channel_2  | int16_t  | / | -660 to +660. Right-Upward is orientation. |
  | channel_3 | int16_t | / | -660 to +660. Right-Upward is orientation. |
  | wheel  | int16_t  | / | -660 to +660. Counter-clockwise is orientation. |


## Sample
#### Logic

- What happenes if the Chassis doesn't receive any control messages within 500ms?
  1. The Chassis disable
  2. The Servos disable
   
- When OrangePi want to control chassis
  1. OrangePi send **chs_manage_info** to enable chassis
  2. OrangePi send **chs_ctl_info** periodically to control chassis
  3. Chassis send **chs_odm_info** as feedback

- When OrangePi want to control servos
  1. OrangePi send **chs_manage_info** to enable servo
  2. OrangePi send **chs_servos_info** periodically to control servos


#### Sample C code

``` C
/*演示OrangePi发送底盘速度控制包*/
#include "mavlink.h" //首先引用头文件
uint32_t cnt = 0;    // 心跳包的数据: 发送次数
/*这是串口发送函数*/
extern void uart_transmit(uint8_t *buffer, uint16_t data_length);

uint8_t MyTask() {
  /*准备各个变量*/
  /*创建一个Mavlink消息结构体*/
  mavlink_message_t *msg =
      (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
  /*清理这部分内存*/
  memset(msg, 0, sizeof(mavlink_message_t));
  /*消息需要发送的字节数长度*/
  uint16_t Txlen = 0;

  /*发送一个正确心跳包*/
  /*准备发送数据*/
  /*获取一段内存用于发送缓冲*/
  /*真实字节长度PAYLOAD_LEN+12*/
  uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

  /*封包*/
  /*各个变量可能使用了约定的枚举, 有最大最小值限制等. FishChassis.xml原始文件*/
  /*函数传入值: 1.发送的主机(Server) 2.发送的部件(Server)*/
  /*函数传入值: 3.消息结构体*/
  /*函数传入值: 4.消息的内容*/
  mavlink_msg_chs_ctrl_info_pack(
    CHS_SYSTEM_ID::CHS_ID_ORANGE,
    CHS_SYSTEM_ID::CHS_ID_ORANGE, 
    msg, 
    0.5f,0.5f, 0.5f);

  /*将包转移至发送缓冲区*/
  Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

  /*执行发送*/
  uart_transmit(txbuf, Txlen);
  /*释放内存*/
  free((void *)txbuf);
  free((void *)msg);
  /*Do something else*/
  /*...*/

  return 1;
}
```

```C
/*演示OrangePi接收来自Chassis的反馈数据*/
#include "mavlink.h" //首先引用头文件

/*这是串口接收函数*/
extern void uart_recieve(uint8_t *&buffer, uint16_t *recieve_length);

uint8_t MyTask() {
  /*准备变量*/
  /*接受到的消息*/
  mavlink_chs_odom_info_t chassis_feedback;

  /*.....*/
  /*创建Mavlink状态变量*/
  mavlink_status_t status;
  /*选择一个Mavlink通道*/
  int chan = MAVLINK_COMM_0;
  /*创建一个Mavlink消息结构体*/
  mavlink_message_t *msg =
      (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
  /*清理这部分内存*/
  memset(msg, 0, sizeof(mavlink_message_t));

  /*接收缓冲区*/
  /*大小随意 注意可能出现多条消息粘包接收情况*/
  uint8_t *rxbuf = (uint8_t *)malloc(512);
  /*接收到的数据长度*/
  uint16_t RxLen = 0;
  /*接收与解析消息*/
  for (;;) {
    /*接收数据*/
    uart_recieve(rxbuf, &RxLen);

    /*收到新数据*/
    for (ULONG i = 0; i < RxLen; i++) {
      /*解包*/
      /*MavlinkV2出现错误包后，再次接收二个正常包后恢复正常解析，但第一个正常包将丢失，第二个可被正确解析*/
      if (mavlink_parse_char(chan, rxbuf[i], msg, &status)) {
        /*解析包成功 处理数据*/
        switch (msg->msgid) {
          case CHS_MOTOR_INFO: {
            mavlink_msg_chs_odom_info_decode(msg, &chassis_feedback);
            /*Do someting with new message*/
            /*...*/
            break;
          }
            /*其他数据处理*/
            /*...*/
        }
      }
    }

    /* Sleep thread for 1ms if no data received */
    sleep(1);
  }

  /*Do something else*/
  /*...*/

  /*释放资源*/
  free((void *)msg);
  free((void *)rxbuf);
  return 1;
}
```