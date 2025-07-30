#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 版权所有声明部分（略）
#
# 核心功能实现代码

import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# 1. 无人机连接配置
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
logging.basicConfig(level=logging.ERROR)  # 仅输出错误日志


# 2. 参数更新回调函数
def param_stab_est_callback(name, value):
    print(f'无人机参数 {name} 已更新为: {value}')


# 3. 异步参数操作函数
def simple_param_async(scf, groupstr, namestr):
    """
    异步设置和监听无人机参数
    scf: SyncCrazyflie连接对象
    groupstr: 参数组名（如'stabilizer'）
    namestr: 参数名（如'estimator'）
    """
    cf = scf.cf  # 获取Crazyflie对象
    full_name = f'{groupstr}.{namestr}'  # 组合完整参数名

    # 添加参数更新回调
    cf.param.add_update_callback(group=groupstr, name=namestr,
                                 cb=param_stab_est_callback)

    # 异步设置参数值并监听变化
    time.sleep(1)
    cf.param.set_value(full_name, 2)  # 设置参数值为2
    time.sleep(1)
    cf.param.set_value(full_name, 1)  # 设置参数值为1
    time.sleep(1)


# 4. 日志数据回调函数
def log_stab_callback(timestamp, data, logconf):
    """
    处理接收到的日志数据
    timestamp: 时间戳
    data: 日志数据（字典）
    logconf: 日志配置对象
    """
    print(f'[{timestamp}][{logconf.name}]: {data}')


# 5. 异步日志记录函数
def simple_log_async(scf, logconf):
    """
    异步日志记录实现
    scf: SyncCrazyflie连接对象
    logconf: LogConfig日志配置对象
    """
    cf = scf.cf
    cf.log.add_config(logconf)  # 添加日志配置
    logconf.data_received_cb.add_callback(log_stab_callback)  # 添加回调
    logconf.start()  # 启动日志记录
    time.sleep(5)  # 持续记录5秒
    logconf.stop()  # 停止日志


# 6. 同步日志记录函数
def simple_log(scf, logconf):
    """
    同步日志记录实现（使用上下文管理器）
    """
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:  # 迭代获取每条日志
            timestamp, data, logconf_name = log_entry
            print(f'[{timestamp}][{logconf_name}]: {data}')
            break  # 只获取第一条日志就退出


# 7. 简单连接测试函数
def simple_connect():
    """基础连接/断开功能演示"""
    print("连接成功!")
    time.sleep(3)  # 维持连接3秒
    print("断开连接")


# 8. 主程序入口
if __name__ == '__main__':
    # 初始化底层驱动
    cflib.crtp.init_drivers()

    # 创建日志配置：记录稳定器的姿态角
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')  # 滚转角
    lg_stab.add_variable('stabilizer.pitch', 'float')  # 俯仰角
    lg_stab.add_variable('stabilizer.yaw', 'float')  # 偏航角

    # 定义要操作的参数组和参数名
    param_group = 'stabilizer'
    param_name = 'estimator'

    # 建立同步连接
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # 根据需要激活不同的功能：
        # simple_connect()  # 基本连接测试
        #simple_log(scf, lg_stab)  # 同步日志
        simple_log_async(scf, lg_stab)  # 异步日志

        # 实际执行的功能：异步参数操作
        # simple_param_async(scf, param_group, param_name)
