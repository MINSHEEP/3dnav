import serial
import time

class RS485Communicator:
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        初始化RS485通信
        :param port: 串口号（如'COM3'或'/dev/ttyUSB0'）
        :param baudrate: 波特率（默认9600）
        :param timeout: 超时时间（秒）
        """
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

    def open(self):
        """打开串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,  # 校验位（根据设备要求修改）
                stopbits=serial.STOPBITS_ONE,  # 停止位（根据设备要求修改）
                bytesize=serial.EIGHTBITS,   # 数据位（通常8位）
                timeout=self.timeout
            )
            if self.serial_port.is_open:
                print(f"串口 {self.port} 已打开")
                return True
            return False
        except Exception as e:
            print(f"打开串口失败：{e}")
            return False

    def send_command(self, command, is_hex=False):
        """
        发送指令
        :param command: 指令内容（字符串或十六进制列表）
        :param is_hex: 是否为十六进制指令（True则command应为列表，如[0x01, 0x03]）
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("串口未打开，无法发送指令")
            return False

        try:
            if is_hex:
                # 发送十六进制指令（需转换为字节）
                send_data = bytes(command)
            else:
                # 发送ASCII字符串指令
                send_data = command.encode('utf-8')

            self.serial_port.write(send_data)
            # print(f"已发送指令：{send_data.hex() if is_hex else command}")
            return True
        except Exception as e:
            print(f"发送指令失败：{e}")
            return False

    def receive_response(self, expected_length=None, is_hex=True):
        """
        接收响应
        :param expected_length: 预期接收的字节数（None则接收所有可读取数据）
        :param is_hex: 是否以十六进制格式返回（False则返回字符串）
        :return: 响应数据
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("串口未打开，无法接收数据")
            return None

        try:
            if expected_length:
                # 读取指定长度的字节
                response = self.serial_port.read(expected_length)
            else:
                # 读取所有可读取的数据（最多等待timeout时间）
                response = self.serial_port.read_all()
                # 若数据为空，尝试再次读取（应对延迟）
                if not response:
                    time.sleep(0.1)
                    response = self.serial_port.read_all()

            if not response:
                print("未接收到响应")
                return None

            if is_hex:
                # 转换为十六进制字符串（空格分隔）
                hex_response = ' '.join(f'{b:02x}' for b in response)
                print(f"接收到响应（十六进制）：{hex_response}")
                return hex_response
            else:
                # 转换为字符串（适用于ASCII协议）
                str_response = response.decode('utf-8', errors='ignore')
                print(f"接收到响应（字符串）：{str_response}")
                return str_response
        except Exception as e:
            print(f"接收响应失败：{e}")
            return None

    def close(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print(f"串口 {self.port} 已关闭")

def calculate_crc8(data):
    """
    crc和校验码
    """
    crc = 0x00
    for byte in data:
        crc += byte
    crc &= 0xFF  # 保持为8位
    return crc

def negative_to_16bit_hex(value):
    if value >= 0:
        return f"0x{value:04X}"  # 非负数直接转换
    # 计算16位补码
    complement = 0x10000 + value
    return f"0x{complement:04X}"

def vel_convert(value):
    complement_16bit = 0x00
    if value >=0:
        complement_16bit = value
    else:
        # 16位补码计算：2^16 + value
        complement_16bit = 0x10000 + value
    # 分离高8位和低8位
    high_byte = (complement_16bit >> 8) & 0xFF  # 右移8位并掩码取低8位
    low_byte = complement_16bit & 0xFF
    return (high_byte, low_byte)

# 使用示例
if __name__ == "__main__":
    # 初始化通信器（根据实际串口号修改）
    # Windows通常为'COM3'，Linux为'/dev/ttyUSB0'
    comm = RS485Communicator(port='/dev/ttyUSB0', baudrate=115200)

    # 打开串口
    if comm.open():
        try:
            liner_vel = -200

            liner_vel_h, liner_vel_l = vel_convert(liner_vel)

            # 示例1：发送十六进制指令（如Modbus协议：读取设备01的0000地址，长度0001）
            # hex_command = [0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A]
            hex_command_front = [0xAA, 0x55, 0x01, 0x03, 0x04, liner_vel_h, liner_vel_l, 0x00, 0xC8, 0x02, 0x00, 0xD1, 0xCC]
            hex_command_front[11] = calculate_crc8(hex_command_front[:11])

            hex_command_back = [0xAA, 0x55, 0x02, 0x03, 0x04, liner_vel_h, liner_vel_l, 0x00, 0xC8, 0x02, 0x00, 0xD1, 0xCC]
            hex_command_back[11] = calculate_crc8(hex_command_back[:11])

            while True:
                comm.send_command(hex_command_front, is_hex=True)
                time.sleep(0.050)
                comm.send_command(hex_command_back, is_hex=True)
                time.sleep(0.100)
        except KeyboardInterrupt:
            # 关闭串口
            comm.close()

        
