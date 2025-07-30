import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

# 无人机参数设置
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')  # 无人机URI
TARGET_POSITION = (1.0, 0.5, 1.0)  # 目标位置 (x, y, z) 米
TARGET_PITCH = 15.0  # 目标俯仰角 度
HOVER_DURATION = 3.0  # 悬停时间 秒
PITCH_DURATION = 5.0  # 保持俯仰角时间 秒
CONTROL_FREQ = 100  # 控制频率 Hz
BASE_THRUST = 38000  # 基础推力 (根据电池调整)


# 欧拉角转四元数
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)


# 主控制函数
def fly_fullstate(scf):
    cf = scf.cf

    # 阶段1: 位置模式飞往目标点
    with PositionHlCommander(cf, default_height=1.0, controller=PositionHlCommander.CONTROLLER_PID) as pc:
        print("飞往目标位置 (x={}, y={}, z={})".format(*TARGET_POSITION))
        pc.go_to(*TARGET_POSITION)
        print("抵达目标位置，悬停 {} 秒".format(HOVER_DURATION))
        time.sleep(HOVER_DURATION)

    # 阶段2: 全状态模式控制俯仰角
    print("切换到全状态模式，设定俯仰角 {} 度".format(TARGET_PITCH))
    start_time = time.time()
    control_period = 1.0 / CONTROL_FREQ

    try:
        # 获取当前姿态
        state = cf.state_estimator.get_state()
        current_yaw = state['yaw']

        while time.time() - start_time < PITCH_DURATION:
            # 转换为弧度
            pitch_rad = math.radians(TARGET_PITCH)

            # 计算目标四元数 (保持当前偏航角)
            qx, qy, qz, qw = euler_to_quaternion(0, pitch_rad, current_yaw)

            # 发送全状态指令
            cf.commander.send_fullstate(
                pos_x=TARGET_POSITION[0],  # 保持目标位置
                pos_y=TARGET_POSITION[1],
                pos_z=TARGET_POSITION[2],
                vx=0.0, vy=0.0, vz=0.0,  # 零速度
                acc_x=0.0, acc_y=0.0, acc_z=0.0,  # 零加速度
                qx=qx, qy=qy, qz=qz, qw=qw,  # 目标姿态
                roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0  # 零角速度
            )

            # 保持控制频率
            time.sleep(control_period)

        # 停止命令
        cf.commander.send_stop_setpoint()
        print("控制完成，准备降落")

    except Exception as e:
        print(f"控制错误: {e}")
        cf.commander.send_stop_setpoint()
        raise


# 主程序
if __name__ == '__main__':
    # 初始化驱动
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print("连接无人机: {}".format(URI))
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("连接成功!")
        fly_fullstate(scf)

    print("任务完成!")
