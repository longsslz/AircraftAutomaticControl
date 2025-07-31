"""
使用键盘控制无人机油门的完整方案
- 上箭头：增加油门
- 下箭头：减少油门
- ESC键：安全退出程序
"""
import threading
import time
import sys
from pynput import keyboard
import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

class DroneThrustControl:
    def __init__(self, link_uri):
        # 连接参数
        self.uri = link_uri
        self._cf = Crazyflie(rw_cache='./cache')
        
        # 油门控制参数
        self.thrust = 20000  # 初始油门值（20%）
        self.thrust_step = 500  # 油门步进值
        self.MIN_THRUST = 10000  # 最小油门（20%）
        self.MAX_THRUST = 60000  # 最大油门（60%）
        
        # 状态标志
        self._is_connected = False
        self._is_running = False
        self._control_thread = None
        self._listener = None
        
        # 注册回调
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        
        print(f'正在连接到 {link_uri}...')
        self._cf.open_link(link_uri)
    
    def _connected(self, link_uri):
        """成功连接后的回调"""
        print(f'成功连接到 {link_uri}')
        self._is_connected = True
        
        # 解锁无人机
        if hasattr(self._cf.platform, 'send_arming_request'):
            self._cf.platform.send_arming_request(True)
        time.sleep(1)
        
        # 启动控制线程
        self._is_running = True
        self._control_thread = threading.Thread(target=self._control_loop)
        self._control_thread.daemon = True
        self._control_thread.start()
        
        # 启动键盘监听
        self._start_keyboard_listener()
        
        print("\n控制说明:")
        print("↑ : 增加油门")
        print("↓ : 减少油门")
        print("ESC : 退出程序")
    
    def _start_keyboard_listener(self):
        """启动键盘监听器"""
        self._listener = keyboard.Listener(on_press=self._on_key_press)
        self._listener.daemon = True
        self._listener.start()
    
    def _on_key_press(self, key):
        """键盘按键处理"""
        if not self._is_running or not self._is_connected:
            return
            
        try:
            if key == keyboard.Key.up:
                # 增加油门
                self.thrust = min(self.thrust + self.thrust_step, self.MAX_THRUST)
                print(f"油门增加至: {self.thrust}")
                
            elif key == keyboard.Key.down:
                # 减少油门
                self.thrust = max(self.thrust - self.thrust_step, self.MIN_THRUST)
                print(f"油门减少至: {self.thrust}")
                
            elif key == keyboard.Key.esc:
                # 退出程序
                print("正在退出...")
                self._quit_program()
        except AttributeError:
            # 忽略特殊按键
            pass
    
    def _control_loop(self):
        """油门控制循环"""
        try:
            # 初始解锁保护
            self._send_stop_command(5)
            
            # 主控制循环
            while self._is_running:
                # 发送当前油门值（roll/pitch/yaw=0）
                self._cf.commander.send_setpoint(0, 0, 0, self.thrust)
                time.sleep(0.05)  # 20Hz控制频率
                
        except Exception as e:
            print(f"控制循环错误: {e}")
            
        finally:
            # 安全停止
            self._safe_shutdown()
    
    def _safe_shutdown(self):
        """安全关闭无人机和连接"""
        print("执行安全关闭...")
        
        # 1. 发送停止命令
        self._send_stop_command(30)
        
        # 2. 关闭连接
        if self._is_connected:
            try:
                self._cf.close_link()
            except Exception as e:
                print(f"关闭连接时出错: {e}")
        
        # 3. 停止线程
        self._is_running = False
        self._is_connected = False
        
        # 4. 停止键盘监听
        if self._listener and self._listener.running:
            self._listener.stop()
    
    def _send_stop_command(self, count):
        """发送指定次数的停止命令"""
        for _ in range(count):
            try:
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)
            except Exception:
                break
    
    def _quit_program(self):
        """退出程序"""
        self._is_running = False
        
        # 等待控制线程结束
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        
        # 退出程序
        print("程序退出")
        sys.exit(0)
    
    def _connection_failed(self, link_uri, msg):
        print(f'连接失败: {link_uri}, 原因: {msg}')
        self._safe_shutdown()
    
    def _connection_lost(self, link_uri, msg):
        print(f'连接丢失: {link_uri}, 原因: {msg}')
        self._safe_shutdown()
    
    def _disconnected(self, link_uri):
        print(f'已断开连接: {link_uri}')
        self._is_connected = False
        self._is_running = False

if __name__ == '__main__':
    # 初始化驱动程序
    cflib.crtp.init_drivers()
    
    # 设置URI（这里使用默认URI，替换为您的实际URI）
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    
    # 启动无人机控制
    controller = DroneThrustControl(uri)
    
    # 主线程等待（直到用户退出）
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller._quit_program()
