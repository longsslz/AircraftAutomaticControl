# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.high_level_commander import HighLevelCommander

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def initialize_logging(scf):
    """初始化姿态日志配置"""
    log_conf = LogConfig(name='Attitude', period_in_ms=50)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')
    log_conf.add_variable('stabilizer.pitch', 'float')
    log_conf.add_variable('stabilizer.roll', 'float')
    
    scf.cf.log.add_config(log_conf)
    return log_conf

def fly_to_target_position(mc, target_x, target_y, target_z):
    """飞行到目标位置"""
    print(f"飞往目标位置 (x={target_x}, y={target_y}, z={target_z})")
    mc.move_to(target_x, target_y, target_z, velocity=0.5)
    
    # 检查是否到达目标位置
    tolerance = 0.1
    while True:
        current_x = mc._x
        current_y = mc._y
        current_z = mc._z
        distance = np.sqrt((target_x - current_x)**2 + 
                           (target_y - current_y)**2 + 
                           (target_z - current_z)**2)
        
        if distance < tolerance:
            print("抵达目标位置")
            break
        time.sleep(0.1)

def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角转换为四元数"""
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    return rotation.as_quat()  # 返回 [x, y, z, w]

def send_full_state_setpoint(scf, target_x, target_y, target_z, target_pitch):
    """发送全状态设定点"""
    commander = HighLevelCommander(scf)
    cf = scf.cf
    
    # 获取当前姿态
    with SyncLogger(scf, cf.log_config) as logger:
        log_entry = next(logger)
        data = log_entry[1]
        current_roll = data['stabilizer.roll']
        current_yaw = data['stabilizer.yaw']
    
    # 创建目标姿态（滚转保持不变，偏航保持不变）
    target_roll = current_roll
    target_yaw = current_yaw
    
    # 转换为四元数
    q = euler_to_quaternion(target_roll, target_pitch, target_yaw)
    
    # 速度设置为零（悬停）
    vx, vy, vz = 0.0, 0.0, 0.0
    
    # 加速度设置为零
    ax, ay, az = 0.0, 0.0, 0.0
    
    # 角速度设置为零
    rate_roll, rate_pitch, rate_yaw = 0.0, 0.0, 0.0
    
    # 发送全状态设定点
    commander.send_full_state_setpoint(
        target_x, target_y, target_z,
        vx, vy, vz,
        ax, ay, az,
        q[0], q[1], q[2], q[3],
        rate_roll, rate_pitch, rate_yaw
    )

def main():
    # 初始化驱动程序
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # 初始化日志配置
        log_conf = initialize_logging(scf)
        scf.cf.log_config = log_conf
        log_conf.start()
        
        # 定义目标位置
        target_x, target_y, target_z = 1.0, 0.5, 1.0
        
        # 使用MotionCommander飞行到目标位置
        with MotionCommander(scf) as mc:
            # 起飞到目标高度
            mc.take_off(target_z, velocity=0.5)
            time.sleep(1)
            
            # 飞往目标位置
            fly_to_target_position(mc, target_x, target_y, target_z)
            
            # 悬停3秒
            print("抵达目标位置，悬停 3.0 秒")
            time.sleep(3.0)
            
            # 切换到全状态模式，设定俯仰角15度
            target_pitch = 15.0
            print(f"切换到全状态模式，设定俯仰角 {target_pitch} 度")
            
            # 持续发送全状态设定点
            start_time = time.time()
            duration = 5.0  # 控制5秒
            
            try:
                while time.time() - start_time < duration:
                    send_full_state_setpoint(
                        scf, 
                        target_x, 
                        target_y, 
                        target_z, 
                        target_pitch
                    )
                    time.sleep(0.05)  # 20Hz控制频率
            except Exception as e:
                print(f"控制错误: {str(e)}")
            
            # 恢复水平姿态
            print("恢复水平姿态")
            for _ in range(20):
                send_full_state_setpoint(scf, target_x, target_y, target_z, 0.0)
                time.sleep(0.05)
            
            print("降落...")
            mc.land()
        
        log_conf.stop()

if __name__ == '__main__':
    main()