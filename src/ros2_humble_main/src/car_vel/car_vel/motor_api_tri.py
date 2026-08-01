import threading
from .rs485_communicator import RS485Communicator, vel_convert, calculate_crc8
import numpy as np
import time

wheel_radius = 105  # mm
l = 344.5  # distance from wheel to center, mm
# Sign conventions / wiring fixes
VY_SIGN = -1.0
OMEGA_SIGN = -1.0
M1_DIR = 1.0
M2_DIR = -1.0
M3_DIR = 1.0


class MotorAPI:
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 115200):
        self.vx = 0
        self.vy = 0
        self.vtheta = 0
        self.comm = RS485Communicator(port, baudrate)
        if not self.comm.open():
            raise ConnectionError(f"Failed to open serial port [{port}]")

    def calculate_tri_speeds(self, vx, vy, omega):
        # Wheel rolling direction angles (radians) relative to +x
        theta1 = np.deg2rad(90)   # M1
        theta2 = np.deg2rad(-150)   # M2
        theta3 = np.deg2rad(-30)  # M3
        # 打印vx
        # self.get_logger().info('aaaaa')
        eps = 100
        if abs(vy) < eps and abs(omega) < 0.05:
            # Pure forward/backward: M1 stays still, M2/M3 drive
            v1 = 0
            v2 = vx
            v3 = vx
            return v1, v2, v3

        vy = VY_SIGN * vy
        omega = OMEGA_SIGN * omega

        v1 = np.cos(theta1) * vx + np.sin(theta1) * vy + l * omega
        v2 = np.cos(theta2) * vx + np.sin(theta2) * vy + l * omega
        v3 = np.cos(theta3) * vx + np.sin(theta3) * vy + l * omega
        return v1 * M1_DIR, v2 * M2_DIR, v3 * M3_DIR

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
        m1, m2, m3 = self.calculate_tri_speeds(self.vx, self.vy, self.vtheta)

        m1_h, m1_l = vel_convert(int(m1))
        m2_h, m2_l = vel_convert(int(m2))
        m3_h, m3_l = vel_convert(int(m3))

        # Front controller: M1 only, second channel set to 0
        hex_command_front = [0xAA, 0x55, 0x01, 0x03, 0x04, 0x00, 0x00, m1_h, m1_l, 0x02, 0x00, 0x00, 0xCC]
        hex_command_front[11] = calculate_crc8(hex_command_front[:11])

        # Back controller: M2, M3
        hex_command_back = [0xAA, 0x55, 0x02, 0x03, 0x04, m2_h, m2_l, m3_h, m3_l, 0x02, 0x00, 0x00, 0xCC]
        hex_command_back[11] = calculate_crc8(hex_command_back[:11])

        self.comm.send_command(hex_command_front, is_hex=True)
        time.sleep(0.10)
        self.comm.send_command(hex_command_back, is_hex=True)

    def set_vel(self, vx, vy, vtheta):
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta

    def __del__(self):
        self.comm.close()

def main():
    port = '/dev/ttyUSB0'
    baudrate = 115200

    motor = MotorAPI(port, baudrate)
    motor.set_vel(100, 0, 0)
    motor.start()
    time.sleep(10)

if __name__ == "__main__":
    main()


