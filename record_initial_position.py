#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_initial_position.py

切换到 Extended Position Control 模式后，读取指定 Dynamixel 电机的当前位置，
保存到 init_pos.txt。保存格式：每行 "<ID>,<Position>"
"""
import time
from dynamixel_sdk import PortHandler, PacketHandler

# ----------------------- 用户配置 -----------------------
DEVICENAME             = '/dev/ttyUSB0'        # 串口号
BAUDRATE               = 1000000
PROTOCOL_VERSION       = 2.0
DXL_IDS                = [1, 2]        # 电机 ID 列表
ADDR_OPERATING_MODE    = 11            # 控制表：操作模式地址
ADDR_TORQUE_ENABLE     = 64            # 控制表：扭矩使能地址
ADDR_PRESENT_POSITION  = 132           # 控制表：当前位置地址
TORQUE_ENABLE          = 1
TORQUE_DISABLE         = 0
MODE_EXT_POSITION_CTRL = 4             # 扩展位置控制模式

# ----------------------- 初始化串口和协议 -----------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    raise IOError(f"无法打开串口 {DEVICENAME}")
if not portHandler.setBaudRate(BAUDRATE):
    raise IOError(f"无法设置波特率 {BAUDRATE}")

# ----------------------- 切换到 Extended Position Control 模式 -----------------------
for dxl_id in DXL_IDS:
    # 先禁用扭矩
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    # 写入扩展位置控制模式
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, MODE_EXT_POSITION_CTRL)
    # 重新启用扭矩
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# 给电机一点时间切换模式
time.sleep(0.1)

# ----------------------- 读取当前位置 -----------------------
positions = {}
for dxl_id in DXL_IDS:
    pos, comm_result, error = packetHandler.read4ByteTxRx(
        portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if comm_result != 0:
        print(f"[ID:{dxl_id}] 通信失败：", packetHandler.getTxRxResult(comm_result))
        continue
    if error != 0:
        print(f"[ID:{dxl_id}] 电机返回错误：", packetHandler.getRxPacketError(error))
        continue
    positions[dxl_id] = pos
    print(f"[ID:{dxl_id}] 当前位置：{pos}")

# ----------------------- 保存到文件 -----------------------
with open('init_pos.txt', 'w', encoding='utf-8') as f:
    for dxl_id, pos in positions.items():
        f.write(f"{dxl_id},{pos}\n")
print("已将初始位置保存到 init_pos.txt：", positions)

# ----------------------- 清理 -----------------------
portHandler.closePort()
