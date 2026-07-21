"""
SPDX-License-Identifier: MIT
Copyright (c) 2024 The FOC Firmware Contributors

生成 UART 串口控制指令帧（16 进制格式）

格式：
  [0xAA][0x08][8 bytes payload][CRC16-CCITT LE]

8 bytes payload:
  [0]    mode - 运行模式 (0=待机, 3=位置, 2=速度, 4=校准)
  [1~4]  target - float 小端，单位弧度
  [5~6]  kp - uint16 小端，当前 kp=5.0 → 32767 (0x7FFF)
  [7]    kd - uint8，当前 kd=0.0 → 0

用法：
  python gen_uart_cmd.py                # 打印预置命令
  python gen_uart_cmd.py --mode 3 --deg 45.0   # 自定义
"""

import struct
import argparse

KP_DEC_SCALE = 0.00015259


def crc16_ccitt(data: bytes) -> int:
    """软件 CRC16-CCITT 逐位法（与单片机代码一致）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_frame(mode: int, target_deg: float, kp: float = 5.0, kd: float = 0.0) -> bytes:
    """构建完整串口指令帧"""
    target_rad = target_deg * 3.1415926535 / 180.0
    kp_raw = int(kp / KP_DEC_SCALE + 0.5)  # 四舍五入
    kd_raw = int(kd / 0.00392157 + 0.5)

    payload = bytearray(8)
    payload[0] = mode
    payload[1:5] = struct.pack('<f', target_rad)
    payload[5:7] = struct.pack('<H', kp_raw)
    payload[7] = kd_raw & 0xFF

    frame = b'\xAA\x08' + bytes(payload)
    crc = crc16_ccitt(frame)
    frame += struct.pack('<H', crc)
    return frame


def print_cmd(name: str, frame: bytes):
    hex_str = ' '.join(f'{b:02X}' for b in frame)
    print(f'{name:>8s}: {hex_str}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='生成 UART 串口控制指令')
    parser.add_argument('--mode', type=int, help='运行模式 (0/2/3/4)')
    parser.add_argument('--deg', type=float, help='目标角度（度）')
    parser.add_argument('--kp', type=float, default=5.0, help='Kp 增益')
    parser.add_argument('--kd', type=float, default=0.0, help='Kd 增益')
    parser.add_argument('--rpm', type=float, help='速度模式目标转速 (rpm)')
    args = parser.parse_args()

    if args.mode is not None:
        if args.mode == 2 and args.rpm is not None:
            # 速度模式：rpm → rad/s
            target_rad_s = args.rpm * 2.0 * 3.1415926535 / 60.0
            frame = build_frame(2, 0.0)
            # 直接替换 target 字段为 rad/s
            frame = bytearray(frame)
            frame[2] = 2  # mode
            frame[3:7] = struct.pack('<f', target_rad_s)
            frame[7:9] = struct.pack('<H', int(5.0 / KP_DEC_SCALE + 0.5))
            frame[9] = 0
            crc = crc16_ccitt(bytes(frame[:10]))
            frame[10:12] = struct.pack('<H', crc)
            print_cmd(f'{args.rpm}rpm', bytes(frame))
        elif args.mode == 3 and args.deg is not None:
            frame = build_frame(3, args.deg, args.kp, args.kd)
            print_cmd(f'{args.deg}°', frame)
        else:
            frame = build_frame(args.mode, args.deg or 0.0)
            print_cmd(f'mode={args.mode}', frame)
    else:
        # 打印预置常用命令
        print('===== 常用串口指令（HEX）=====\n')

        cmds = [
            ('待机',    build_frame(0, 0.0)),
            ('0°',      build_frame(3, 0.0)),
            ('10°',     build_frame(3, 10.0)),
            ('45°',     build_frame(3, 45.0)),
            ('90°',     build_frame(3, 90.0)),
            ('校准',    build_frame(4, 0.0)),
        ]

        for name, frame in cmds:
            print_cmd(name, frame)

        print('\n===== 用串口助手发送 =====')
        print('1. 选 HEX 发送')
        print('2. 波特率 115200')
        print('3. 把上面的 HEX 字符串直接粘贴发送')
