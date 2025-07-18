## base control
```bash
roslaunch base_control base_control.launch
python web_car_control.py
```

## details
////////////////////// 通讯协议数据构成 ////////////////////// 串口波特率:115200,1停止位，8数据位，无校验 约定: 1.上位机往下位机发送的消息，功能码为奇数，下位机往上位机发送，功能码为偶数 2.帧长度为整个数据包长度，包括从帧头到校验码全部数据 3.ID为下位机编号，为级联设计预留 4.预留位，为后续协议扩展预留 5.CRC校验为一字节，校验方式为CRC-8/MAXIM 6.线速度单位为m/s，角速度单位为rad/s（弧度制），角度单位为度（角度制）

帧头 帧长度 ID 功能码 数据 预留位 CRC校验 0x5a 0x00 0x01 0x00 0xXX...0xXX 0x00 0xXX

帧头： 1Byte 0x5a 固定值 帧长度： 帧头(1 Byte)+帧长度(1 Byte)+ID(1 Byte)+功能码(1 Byte)+数据(0~250Byte))+预留位(1 Byte)+CRC校验(1 Byte) 功能码： 1Byte控制板端发送的功能码为偶数,控制板端接收的功能码为奇数 数据： 长度和内容具体参照各功能码定义 预留位： 目前设置为0x00，为将来协议可能的扩展预留 CRC校验： 1Byte，校验方式为CRC-8/MAXIM，设置为0xFF，则强制不进行CRC校验 （参考在线计算网站：http://www.ip33.com/crc.html）

上位机-底盘建立连接定义： 在为建立连接的状态下，底盘收到协议内数据，则判定为建立连接，建立连接时，底盘IMU会执行初始化，耗时2S左右，所以建议以不低于2hz的频率向底盘发送指令

上位机-底盘断开连接定义： 在建立连接状态后，超过1000ms没有收到新的协议内数据，底盘判定断开连接，则会主动停止电机运动

功能码:

0x01 上位机向下位机发送速度控制指令，数据长度为6Byte，数据为X轴方向速度1000(int16_t) + Y轴方向速度1000(int16_t) + Z轴角速度*1000(int16_t) 数据格式为： Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 X MSB X LSB Y MSB Y LSB Z MSB Z LSB 例:5A 0C 01 01 01 F4 00 00 00 00 00 56 (底盘以0.5m/s的速度向前运动)

0x02 下位机回复上位机的速度控制指令，数据长度1Byte，仅在速度设置失败时候回复，正常时无回复 数据格式为： Byte1

0x03 上位机向下位机发送速度查询指令，数据长度为0Byte 例:5A 06 01 03 00 DF

0x04 下位机上报当前速度，数据长度为6Byte，数据为X轴方向速度1000(int16_t) + Y轴方向速度1000(int16_t) + Z轴角速度*1000(int16_t) 数据格式为： Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 X MSB X LSB Y MSB Y LSB Z MSB Z LSB

0x05 上位机向下位机查询IMU数据，数据长度为0Byte 例:5A 06 01 05 00 75

0x06 下位机上报当IMU数据，数据长度为6Byte，数据为Pitch1000(int16_t) + Roll1000(int16_t) + Yaw*1000(int16_t) 数据格式为： Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 Pitch MSB Pitch LSB Roll MSB Roll LSB Yaw MSB Yaw LSB

0x07 上位机向下位机查询电池信息，数据长度为0Byte 例:5A 06 01 07 00 E4

0x08 下位机上报电池信息，数据长度为4Byte，数据为电压Voltage1000(uint16_t) + 电流Current1000(uint16_t) 数据格式为： Byte1 Byte2 Byte3 Byte4 Voltage MSB Voltage LSB Current MSB Current LSB

0x09 上位机向下位机获取里程计信息 例:5A 06 01 09 00 38

0x0a 下位机上报速度航向信息，数据长度为6Byte，数据为线速度1000、角度100、角速度*1000 Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 X MSB X LSB Yaw MSB Yaw LSB Z MSB Z LSB

0x11 上位机向下位机获取里程计信息（相比功能码0x09对应的消息，增加了Y轴线速度，为了适应全向移动底盘的需求） 例:5A 06 01 11 00 A2

0x12 下位机上报速度航向信息，数据长度为8Byte，数据为X轴线速度1000、Y轴线速度1000、角度100、角速度1000， Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 Byte7 Byte8 X MSB X LSB Y MSB Y LSB Yaw MSB Yaw LSB Z MSB Z LSB

0x13 上位机向下位机查询IMU原始数据，数据长度为0Byte 5A 06 01 13 00 33

0x14 下位机上报当IMU数据，数据长度为32Byte，数据为GyroX100000(int32_t)、GyroY100000(int32_t)、GyroZ100000(int32_t)、 AccelX100000(int32_t)、AccelY100000(int32_t)、AccelZ100000(int32_t)、 QuatW×10000、QuatX×10000、QuatY×10000、QuatZ×10000 数据格式为：（高位在前，低位在后） Byte1~4 Byte5~8 Byte9~12 Byte13~16 Byte17~20 Byte21~24 Byte25~26 Byte27~28 Byte29~30 Byte31~32 GyroX GyroY GyroZ AccelX AccelY AccelZ QuatW QuatX QuatY QuatZ

0x15 上位机向下位机发送速度控制指令(阿克曼结构车型)，数据长度为6Byte，数据为X轴方向速度1000(int16_t) + X轴方向加速度1000(int16_t) + 转向角度*1000(int16_t)(角度为弧度制) 注：加速度值暂时未使用 Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 X MSB X LSB AX MSB AX LSB A MSB A LSB 例:5A 0C 01 15 00 CB 00 00 00 CB 00 74 (底盘以0.2m/s的速度，0.2弧度的转向角向前运动)

0x17 上位机向下位机获取ADC接口的ADC值，ADC采样范围为0~6.6V，ADC值和电压的换算关系为((电压/2)/3.3V)*4095 = ADC值 5A 06 01 17 00 08

0x18 下位机回复上位机ADC值，数据格式为：（高位在前，低位在后） Byte1~2 Byte3~4 Byte5~6 Byte7~8 Byte9~10 Byt11~12 ADC1 ADC2 ADC3 ADC4 ADC5 ADC6

0x19 上位机向下位机获取超声波测距值，单位为cm 5A 06 01 19 00 D4

0x1a 下位机回复上位机超声波测距值 Byte1 Byte2 Byte3 Byte4 超声波1 超声波2 超声波3 超声波4

0x1b 上位机设置下位机IO口值，数据长度为4Byte，分别对应4个IO口的值，0为低电平，1为高电平 例：5A 0a 01 1B 01 00 00 00 00 93 设置IO1为高电平，IO2、IO3、IO4为低电平

0x1c 下位机回复上位机IO口值 例：5A 0a 01 1C 01 00 00 00 00 16 当前IO1为高电平，IO2、IO3、IO4为低电平

0x1d 上位机设置下位机PWM口输出值，数据长度为8Byte，分别对应4个PWM口的高电平时间，单位为us，PWM周期为20000us Byte1~2 Byte3~4 Byte5~6 Byte7~8 Byte9~10 Byt11~12 PWM1 PWM2 PWM3 PWM4 PWM5 PWM6 例：5A 0e 01 1D 01 00 00 00 00 00 00 00 00 46 设置PWM1为高电平时间为256us，PWM2、PWM3、PWM4高电平时间为0

0x1e 下位机回复上位机PWM口值 例：5A 0e 01 1E 01 00 00 00 00 00 00 00 00 0D 当前PWM1为高电平时间为256us，PWM2、PWM3、PWM4高电平时间为0

0x1f 上位机设置下位机LED灯条灯颜色和亮度，数据长度为15Byte，分别对应5个LED灯的R、G、B值（值范围为0~255） 例：5A 15 01 1F FF 00 00 00 FF 00 00 80 FF 00 00 00 00 00 00 00 4D 设置LED1 红色亮度为255，LED2 绿色亮度为255，LED3绿色亮度为128，蓝色亮度为255，其余LED亮度均为0

0x21 上位机向下位机获取底盘配置信息 例:5a 06 01 21 00 8F

0x22 下位机回复配置信息数据长度为xByte,格式为BASE_TYPE(底盘类型uint8_t) + MOTOR_TYPE(电机型号uint8_t) + ratio10(电机减速比int16_t) + diameter10(轮胎直径int16_t) Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 BASE_TYPE MOTOR_TYPE ratio MSB ratio LSB diameter MSB diameter LSB

0xf1 上位机向下位机查询版本号 例:5a 06 01 f1 00 d7

0xf2 下位机回复版本号，数据长度为6Byte,格式为硬件版本号xx.yy.zz,软件版本号aa.bb.cc， Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 xx yy zz aa bb cc

0xf3 上位机向下位机查询主板SN号 例:5a 06 01 f3 00 46

0xf4 下位机回复版本号，数据长度为12Byte,高位在前，低位在后 Byte1~12 SN号

0xfd 上位机向下位机发送重启指令,下位机无回复 例:5a 06 01 fd 00 9a