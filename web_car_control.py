#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import threading
from geometry_msgs.msg import Twist
from flask import Flask, render_template, jsonify, request, Response
import cv2
import numpy as np

app = Flask(__name__)

# 全局变量
pub = None
is_ros_initialized = False
current_command = None
command_lock = threading.Lock()

# 摄像头相关变量
camera = None
camera_lock = threading.Lock()
camera_initialized = False

def init_ros():
    """初始化ROS节点和发布者"""
    global pub, is_ros_initialized
    
    if not is_ros_initialized:
        try:
            # 检查ROS是否已经初始化
            if not rospy.core.is_initialized():
                rospy.init_node('web_car_control', anonymous=True)
            
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.sleep(0.5)  # 等待发布者连接
            is_ros_initialized = True
            print("ROS节点初始化成功")
        except Exception as e:
            print(f"ROS节点初始化失败: {e}")
            return False
    return True

def send_car_command(linear_x=0.0, linear_y=0.0, angular_z=0.0):
    """
    发送小车控制指令（非阻塞）
    
    Args:
        linear_x (float): X轴方向线速度 (m/s)
        linear_y (float): Y轴方向线速度 (m/s) 
        angular_z (float): Z轴角速度 (rad/s)
    """
    if not init_ros():
        return False
    
    try:
        # 创建Twist消息
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        
        # 发送指令
        pub.publish(msg)
        print(f"发送指令: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
        return True
        
    except Exception as e:
        print(f"发送指令失败: {e}")
        return False

def stop_car():
    """停止小车"""
    return send_car_command(0.0, 0.0, 0.0)

def car_control_thread():
    """小车控制线程"""
    global current_command
    
    while True:
        with command_lock:
            if current_command:
                if current_command == 'forward':
                    send_car_command(linear_x=0.1)
                elif current_command == 'backward':
                    send_car_command(linear_x=-0.1)
                elif current_command == 'turn_left':
                    send_car_command(linear_x=0.1, angular_z=0.5)
                elif current_command == 'turn_right':
                    send_car_command(linear_x=0.1, angular_z=-0.5)
                elif current_command == 'stop':
                    stop_car()
                    current_command = None
            else:
                # 没有命令时停止小车
                stop_car()
        
        time.sleep(0.1)  # 100ms循环

def set_current_command(command):
    """设置当前命令"""
    global current_command
    with command_lock:
        current_command = command

def car_forward():
    """小车前进"""
    set_current_command('forward')
    return True

def car_backward():
    """小车后退"""
    set_current_command('backward')
    return True

def car_turn_left():
    """小车左转前进"""
    set_current_command('turn_left')
    return True

def car_turn_right():
    """小车右转前进"""
    set_current_command('turn_right')
    return True

def car_stop():
    """小车停止"""
    set_current_command('stop')
    return True

def init_camera():
    """初始化摄像头"""
    global camera, camera_initialized
    
    if not camera_initialized:
        try:
            # 尝试打开USB摄像头，通常设备号为0
            camera = cv2.VideoCapture(0)
            
            # 设置摄像头参数
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_FPS, 30)
            
            if camera.isOpened():
                camera_initialized = True
                print("摄像头初始化成功")
                return True
            else:
                print("无法打开摄像头")
                return False
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            return False
    return True

def get_camera_frame():
    """获取摄像头帧"""
    global camera, camera_initialized
    
    if not camera_initialized:
        if not init_camera():
            return None
    
    try:
        with camera_lock:
            if camera and camera.isOpened():
                ret, frame = camera.read()
                if ret:
                    # 压缩图像为JPEG格式
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ret:
                        return buffer.tobytes()
        return None
    except Exception as e:
        print(f"获取摄像头帧失败: {e}")
        return None

def generate_video_frames():
    """生成视频流帧"""
    while True:
        frame_data = get_camera_frame()
        if frame_data:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
        else:
            # 如果无法获取帧，返回一个错误图像
            error_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_frame, 'Camera Not Available', (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', error_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.033)  # 约30FPS

# Flask路由
@app.route('/')
def index():
    """主页"""
    return render_template('car_control.html')

@app.route('/video_feed')
def video_feed():
    """视频流路由"""
    return Response(generate_video_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/forward', methods=['POST'])
def api_forward():
    """前进API - 持续执行"""
    success = car_forward()
    return jsonify({'success': success, 'message': '前进指令已发送' if success else '前进指令发送失败'})

@app.route('/api/backward', methods=['POST'])
def api_backward():
    """后退API - 持续执行"""
    success = car_backward()
    return jsonify({'success': success, 'message': '后退指令已发送' if success else '后退指令发送失败'})

@app.route('/api/turn_left', methods=['POST'])
def api_turn_left():
    """左转API - 持续执行"""
    success = car_turn_left()
    return jsonify({'success': success, 'message': '左转指令已发送' if success else '左转指令发送失败'})

@app.route('/api/turn_right', methods=['POST'])
def api_turn_right():
    """右转API - 持续执行"""
    success = car_turn_right()
    return jsonify({'success': success, 'message': '右转指令已发送' if success else '右转指令发送失败'})

@app.route('/api/stop', methods=['POST'])
def api_stop():
    """停止API"""
    success = car_stop()
    return jsonify({'success': success, 'message': '停止指令已发送' if success else '停止指令发送失败'})

@app.route('/api/status', methods=['GET'])
def api_status():
    """状态API"""
    return jsonify({
        'ros_initialized': is_ros_initialized,
        'status': 'ready' if is_ros_initialized else 'initializing'
    })

if __name__ == '__main__':
    # 在主线程中初始化ROS
    print("正在初始化ROS节点...")
    if init_ros():
        print("ROS节点初始化成功")
    else:
        print("ROS节点初始化失败，但Web服务仍可启动")
    
    # 初始化摄像头
    print("正在初始化摄像头...")
    if init_camera():
        print("摄像头初始化成功")
    else:
        print("摄像头初始化失败，视频功能将不可用")
    
    # 启动小车控制线程
    control_thread = threading.Thread(target=car_control_thread, daemon=True)
    control_thread.start()
    print("小车控制线程已启动")
    
    print("启动Web小车控制服务...")
    print("访问地址: http://localhost:5001")
    print("按 Ctrl+C 停止服务")
    
    # 启动Flask应用
    app.run(host='0.0.0.0', port=5001, debug=False) 