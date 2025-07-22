#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python 版：双 Dynamixel 电机相位差 PID 控制
支持任意键或回车退出，兼容 PyCharm
"""
import time
import threading
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ----------------------- 用户参数 -----------------------
DEVICENAME       = '/dev/ttyUSB0'
BAUDRATE         = 3000000
PROTOCOL_VERSION = 2.0
DXL_IDS          = [1, 2]
POSITION_SCALE   = 0.087891
MAX_VELOCITY     = 235
KP, KI, KD       = 0.04, 0.01, 0.0
CONTROL_PERIOD   = 0.05
DEBUG_PID        = True

#############################
TARGET_VELOCITY  = 200


TARGET_PHASE_DEG =270

TARGET_PHASE     = int(TARGET_PHASE_DEG / POSITION_SCALE)
################################
# ----------------------- 控制表地址 -----------------------
ADDR_TORQUE_ENABLE      = 64
ADDR_OPERATING_MODE     = 11
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_GOAL_VELOCITY      = 104
ADDR_PRESENT_VELOCITY   = 128
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
MODE_EXT_POSITION_CTRL  = 4
MODE_VELOCITY_CTRL      = 1

# ----------------------- 退出标志 -----------------------
stop_flag = False

def wait_for_input():
    global stop_flag
    input(">>> 在任意时候按 Enter 或回车停止程序 …\n")
    stop_flag = True
# 启动守护线程监听回车
threading.Thread(target=wait_for_input, daemon=True).start()

# ----------------------- 初始化 -----------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
if not portHandler.openPort(): raise IOError(f"无法打开端口 {DEVICENAME}")
if not portHandler.setBaudRate(BAUDRATE): raise IOError(f"无法设置波特率 {BAUDRATE}")

# ----------------------- 辅助函数 -----------------------
def to_signed32(val):
    return val - 2**32 if val >= 2**31 else val

def safe_read(dxl_id, address, name):
    raw, result, error = packetHandler.read4ByteTxRx(portHandler, dxl_id, address)
    if result != COMM_SUCCESS:
        print(f"[ID:{dxl_id}] 读取{name}失败：{packetHandler.getTxRxResult(result)}")
        return None
    if error != 0:
        print(f"[ID:{dxl_id}] {name}错误：{packetHandler.getRxPacketError(error)}")
        return None
    return to_signed32(raw)

def read_position(dxl_id): return safe_read(dxl_id, ADDR_PRESENT_POSITION, '位置')

def read_velocity(dxl_id): return safe_read(dxl_id, ADDR_PRESENT_VELOCITY, '速度')

def write_velocity(dxl_id, velocity):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, velocity)

def write_position(dxl_id, position):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)

def set_operating_mode(dxl_id, mode):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, mode)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def set_profile(dxl_id):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, 0)
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, MAX_VELOCITY)

# ----------------------- 步骤 1：模式切换 -----------------------
for dxl_id in DXL_IDS:
    set_operating_mode(dxl_id, MODE_EXT_POSITION_CTRL)
    set_profile(dxl_id)
# 等待模式生效
time.sleep(0.1)

# ----------------------- 步骤 2：读取或保存初始位置 -----------------------
pos_init = {}
try:
    with open('init_pos.txt') as f:
        for line in f:
            did, pos = map(int, line.split(','))
            pos_init[did] = pos
    print("加载初始位置：", pos_init)
except FileNotFoundError:
    for dxl_id in DXL_IDS:
        p = read_position(dxl_id)
        pos_init[dxl_id] = p if p is not None else 0
    with open('init_pos.txt','w') as f:
        for did, pos in pos_init.items(): f.write(f"{did},{pos}\n")
    print("首次保存初始位置：", pos_init)

# ----------------------- 步骤 3：预对齐 -----------------------
pos_now = {i: (read_position(i) or pos_init[i]) for i in DXL_IDS}
counts = ( pos_now[DXL_IDS[0]]-pos_init[DXL_IDS[0]], pos_now[DXL_IDS[1]]-pos_init[DXL_IDS[1]])
delta = TARGET_PHASE - sum(counts)
write_position(DXL_IDS[1], pos_now[DXL_IDS[1]]+delta)
# 等待执行
time.sleep(0.5)

# ----------------------- 步骤 4：速度模式 & 初速 -----------------------
for dxl_id in DXL_IDS: set_operating_mode(dxl_id, MODE_VELOCITY_CTRL)
# 等待模式生效
time.sleep(0.1)
# 同步初速
write_velocity(DXL_IDS[0], TARGET_VELOCITY)
write_velocity(DXL_IDS[1], -TARGET_VELOCITY)
# 等待稳定
time.sleep(1)

# ----------------------- 步骤 5：PID 控制循环 -----------------------
e_prev = i_accum = 0.0
last = time.time()
print("开始 PID 控制，按 Enter 或任意键退出…")
while not stop_flag:
    # kbhit 兼容低延迟退出
    
    # 读取状态
    p1 = read_position(DXL_IDS[0]); p2 = read_position(DXL_IDS[1])
    v1 = read_velocity(DXL_IDS[0]); v2 = read_velocity(DXL_IDS[1])
    # 有读失败则跳过
    if None in (p1, p2, v1, v2): continue
    # 计算相对脉冲差
    c1 = abs(p1 - pos_init[DXL_IDS[0]])  # 保留正负表示相位领先或滞后
    c2 = abs(p2 - pos_init[DXL_IDS[1]])
    print(f"实时脉冲 c1={c1}, c2={c2}")
    e = (c1 - c2) - TARGET_PHASE
    # PID
    i_accum = max(min(i_accum + e * dt, 200), -200)
    d = (e - e_prev) / dt if dt > 0 else 0
    v2_cmd = int(max(min(TARGET_VELOCITY + (KP*e + KI*i_accum + KD*d), MAX_VELOCITY), -MAX_VELOCITY))
    write_velocity(DXL_IDS[1], -v2_cmd)
    e_prev = e
    if DEBUG_PID:
        print(f"e={e:.1f} I={i_accum:.1f} D={d:.1f} v1={v1:.1f} v2cmd/act={v2_cmd}/{v2}")

# ----------------------- 步骤 6：清理 -----------------------
print("退出信号，停止并关闭电机…")
for dxl_id in DXL_IDS:
    write_velocity(dxl_id, 0)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
print("已完成退出和释放。")
