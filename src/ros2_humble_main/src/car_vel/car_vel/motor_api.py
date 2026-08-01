import threading
from .rs485_communicator import RS485Communicator, vel_convert, calculate_crc8
import numpy as np
import time

wheel_radius = 105        # 轮子半径 105mm
l = 250                   # 轮子到中心的x和y方向距离

class MotorAPI:
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 115200):
        # 当前速度mm/s
        self.vx = 0
        self.vy = 0
        # 当前角速度rad/s
        self.vtheta = 0
        # 初始化通信器（根据实际串口号修改）
        # Windows通常为'COM3'，Linux为'/dev/ttyUSB0'
        self.comm = RS485Communicator(port, baudrate)
        # 打开串口
        if not self.comm.open():
            raise ConnectionError(f"打开串口[{port}]失败")
        
    def calculate_omni_speeds(self, vx, vy, omega):
        """
        计算菱形布局全向轮的速度，符合特定的y方向运动符号要求
        
        参数:
            vx: 机器人x方向线速度 (mm/s)，向前为正
            vy: 机器人y方向线速度 (mm/s)，向右为正
            omega: 机器人旋转角速度 (rad/s)，逆时针为正
            wheel_radius: 轮子半径 (mm)
            l: 轮子到中心的距离在x和y方向的分量 (mm)，假设lx=ly=l
            
        返回:
            四个轮子的转速 (rad/s)，顺序为: 右前、左前、右后、左后
        """
        # 修正后的速度解算矩阵，满足：
        # - x正方向：四电机均为正
        # - y正方向：右前、左后为正；左前、右后为负
        conversion_matrix = np.array([
            [ 1/np.sqrt(2),  1/np.sqrt(2), -l],  # 右前轮
            [ 1/np.sqrt(2), -1/np.sqrt(2),  l],  # 左前轮
            [ 1/np.sqrt(2), -1/np.sqrt(2), -l],  # 右后轮
            [ 1/np.sqrt(2),  1/np.sqrt(2),  l]   # 左后轮
        ])
        
        # 机器人运动向量 [vx, vy, omega]
        motion_vector = np.array([vx, vy, -omega])
        
        # 计算每个轮子的线速度 (m/s)
        wheel_linear_speeds = conversion_matrix.dot(motion_vector)

        return wheel_linear_speeds
        
        # 转换为角速度 (rad/s)
        # wheel_angular_speeds = wheel_linear_speeds / wheel_radius

        # return wheel_angular_speeds

    def start(self):
        def run():
            try:
                while True:
                    self.update()
                    time.sleep(0.200)
            except KeyboardInterrupt:
                self.comm.close()
        thread = threading.Thread(target=run)
        thread.daemon = True
        thread.start()

    def update(self):
        # 解算4个轮子的线速度
        rf, lf, rb, lb = self.calculate_omni_speeds(self.vx, self.vy, self.vtheta)
        rf_h, rf_l = vel_convert(int(rf))
        lf_h, lf_l = vel_convert(int(lf))
        rb_h, rb_l = vel_convert(int(rb))
        lb_h, lb_l = vel_convert(int(lb))

        # print(f"右前轮速度: {rf_h} {rf_l}")
        # print(f"左前轮速度: {lf_h} {lf_l}")
        # print(f"右后轮速度: {rb_h} {rb_l}")
        # print(f"左后轮速度: {lb_h} {lb_l}")

        # 示例1：发送十六进制指令（如Modbus协议：读取设备01的0000地址，长度0001）
        # hex_command = [0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A]
        hex_command_front = [0xAA, 0x55, 0x01, 0x03, 0x04, rf_h, rf_l, lf_h, lf_l, 0x02, 0x00, 0x00, 0xCC]
        hex_command_front[11] = calculate_crc8(hex_command_front[:11])

        hex_command_back = [0xAA, 0x55, 0x02, 0x03, 0x04, rb_h, rb_l, lb_h, lb_l, 0x02, 0x00, 0x00, 0xCC]
        hex_command_back[11] = calculate_crc8(hex_command_back[:11])

        self.comm.send_command(hex_command_front, is_hex=True)
        time.sleep(0.10)
        self.comm.send_command(hex_command_back, is_hex=True)

    def set_vel(self, vx, vy, vtheta):
        # 设置当前速度
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta

    def __del__(self):
        self.comm.close()

def main():
    # 配置串口参数
    # Windows系统通常是'COM1', 'COM2'等
    # Linux系统通常是'/dev/ttyUSB0', '/dev/ttyS0'等
    port = '/dev/ttyUSB0'  # 根据实际情况修改
    baudrate = 115200  # 根据设备要求修改
    
    motor = MotorAPI(port, baudrate)
    motor.set_vel(100, 80, 0)
    motor.start()
    time.sleep(10)

if __name__ == "__main__":
    main()