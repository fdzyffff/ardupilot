#!/usr/bin/env python3
"""
YOLO 二进制协议模拟器 —— 用于 SITL 仿真测试 YoloDrop 模块

连接到 SITL 暴露的 TCP 端口，模拟 AI 模块 (yolo_to_fmu) 发送检测数据。

用法:
    python3 yolo_sim.py [--host HOST] [--port PORT] [--mode MODE]

模式:
    idle       - 不发送检测数据（空帧），测试无目标情况
    static     - 发送固定偏移的目标，测试接近逻辑
    centering  - 目标从偏移逐渐移向中心，模拟接近过程
    drop       - 完整流程: 偏移→居中→面积增大→触发抛投
    lost       - 发送几秒后停止，测试目标丢失处理
"""

import socket
import struct
import time
import argparse
import math


def build_frame(detections):
    """
    构建 yolo_to_fmu 二进制帧

    detections: list of dict, 每个包含:
        class_id (int), confidence (float),
        offset_x (float), offset_y (float),
        norm_width (float), norm_height (float)
    """
    n = len(detections)
    count_byte = n & 0xFF
    data = b''
    for det in detections:
        data += struct.pack('<B', det['class_id'])
        data += struct.pack('<f', det['confidence'])
        data += struct.pack('<f', det['offset_x'])
        data += struct.pack('<f', det['offset_y'])
        data += struct.pack('<f', det['norm_width'])
        data += struct.pack('<f', det['norm_height'])

    checksum = count_byte
    for b in data:
        checksum = (checksum + b) & 0xFF

    frame = bytes([0xA5, 0x5A, count_byte]) + data + bytes([checksum, 0xFF])
    return frame


def build_empty_frame():
    return build_frame([])


def make_detection(class_id=0, confidence=0.85,
                   offset_x=0.0, offset_y=0.0,
                   norm_width=0.05, norm_height=0.05):
    return {
        'class_id': class_id,
        'confidence': confidence,
        'offset_x': offset_x,
        'offset_y': offset_y,
        'norm_width': norm_width,
        'norm_height': norm_height,
    }


def run_idle(sock, hz=10):
    """空帧模式: 持续发送无检测结果"""
    print("[idle] 发送空帧, Ctrl+C 退出")
    dt = 1.0 / hz
    while True:
        sock.sendall(build_empty_frame())
        time.sleep(dt)


def run_static(sock, hz=10, ox=0.3, oy=-0.2):
    """静态目标: 固定偏移位置"""
    print(f"[static] 目标偏移 ox={ox}, oy={oy}, Ctrl+C 退出")
    dt = 1.0 / hz
    while True:
        det = make_detection(offset_x=ox, offset_y=oy,
                             norm_width=0.04, norm_height=0.04)
        sock.sendall(build_frame([det]))
        time.sleep(dt)


def run_centering(sock, hz=10, duration=20.0):
    """
    模拟目标从偏移位置逐渐移向中心
    模拟飞机接近目标正上方的过程
    """
    print(f"[centering] 目标从偏移移向中心, 持续 {duration}s")
    dt = 1.0 / hz
    steps = int(duration * hz)
    ox_start, oy_start = 0.4, -0.3
    for i in range(steps):
        t = i / steps
        ox = ox_start * (1.0 - t)
        oy = oy_start * (1.0 - t)
        det = make_detection(offset_x=ox, offset_y=oy,
                             norm_width=0.04, norm_height=0.04)
        sock.sendall(build_frame([det]))
        time.sleep(dt)
    print("[centering] 完成")


def run_drop(sock, hz=10):
    """
    完整抛投流程模拟:
      阶段1: 2s 空帧 (搜索中)
      阶段2: 8s 目标从偏移移向中心 (接近)
      阶段3: 8s 目标居中，面积逐渐增大 (下降)
      阶段4: 10s 目标居中且面积稳定 (悬停+释放+完成)
      阶段5: 持续发送空帧 (任务结束后)
    """
    dt = 1.0 / hz

    # 阶段1: 搜索中, 无目标
    print("[drop] 阶段1: 搜索中 (2s 空帧)")
    for _ in range(int(2 * hz)):
        sock.sendall(build_empty_frame())
        time.sleep(dt)

    # 阶段2: 发现目标, 从偏移接近中心
    print("[drop] 阶段2: 发现目标, 水平接近 (8s)")
    approach_steps = int(8 * hz)
    ox_start, oy_start = 0.35, -0.25
    for i in range(approach_steps):
        t = i / approach_steps
        ox = ox_start * (1.0 - t)
        oy = oy_start * (1.0 - t)
        det = make_detection(offset_x=ox, offset_y=oy,
                             norm_width=0.04, norm_height=0.04)
        sock.sendall(build_frame([det]))
        time.sleep(dt)

    # 阶段3: 居中, 面积逐渐增大 (模拟下降)
    print("[drop] 阶段3: 居中, 下降中 (8s)")
    descend_steps = int(8 * hz)
    area_start = 0.04 * 0.04  # 初始面积
    area_end = 0.10           # 目标面积
    for i in range(descend_steps):
        t = i / descend_steps
        area = area_start + (area_end - area_start) * t
        side = math.sqrt(area)
        ox = 0.01 * math.sin(i * 0.1)   # 微小抖动
        oy = 0.01 * math.cos(i * 0.1)
        det = make_detection(offset_x=ox, offset_y=oy,
                             norm_width=side, norm_height=side)
        sock.sendall(build_frame([det]))
        time.sleep(dt)

    # 阶段4: 稳定悬停, 等待抛投完成
    print("[drop] 阶段4: 稳定悬停, 等待抛投 (10s)")
    hover_steps = int(10 * hz)
    side = math.sqrt(area_end)
    for i in range(hover_steps):
        ox = 0.005 * math.sin(i * 0.05)
        oy = 0.005 * math.cos(i * 0.05)
        det = make_detection(offset_x=ox, offset_y=oy,
                             norm_width=side, norm_height=side)
        sock.sendall(build_frame([det]))
        time.sleep(dt)

    # 阶段5: 任务结束, 发送空帧
    print("[drop] 阶段5: 任务结束, 发送空帧")
    for _ in range(int(5 * hz)):
        sock.sendall(build_empty_frame())
        time.sleep(dt)

    print("[drop] 模拟完成")


def run_lost(sock, hz=10):
    """
    目标丢失测试:
      3s 有目标 → 5s 无数据 → 3s 有目标
    """
    dt = 1.0 / hz

    print("[lost] 阶段1: 有目标 (3s)")
    for _ in range(int(3 * hz)):
        det = make_detection(offset_x=0.2, offset_y=-0.1,
                             norm_width=0.04, norm_height=0.04)
        sock.sendall(build_frame([det]))
        time.sleep(dt)

    print("[lost] 阶段2: 目标丢失, 停止发送 (5s)")
    time.sleep(5.0)

    print("[lost] 阶段3: 目标恢复 (3s)")
    for _ in range(int(3 * hz)):
        det = make_detection(offset_x=0.15, offset_y=-0.08,
                             norm_width=0.04, norm_height=0.04)
        sock.sendall(build_frame([det]))
        time.sleep(dt)

    print("[lost] 模拟完成")


def main():
    parser = argparse.ArgumentParser(description='YOLO SITL 模拟器')
    parser.add_argument('--host', default='127.0.0.1', help='SITL TCP 地址')
    parser.add_argument('--port', type=int, default=5770, help='SITL 串口 TCP 端口')
    parser.add_argument('--hz', type=int, default=10, help='发送频率 (Hz)')
    parser.add_argument('--mode', default='drop',
                        choices=['idle', 'static', 'centering', 'drop', 'lost'],
                        help='模拟模式')
    args = parser.parse_args()

    print(f"连接 {args.host}:{args.port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except ConnectionRefusedError:
        print(f"连接失败! 请确认 SITL 已启动且串口端口 {args.port} 已开放")
        print(f"SITL 启动参数示例: sim_vehicle.py ... -A '--serial5=tcp:{args.port}'")
        return

    print(f"已连接, 模式={args.mode}, 频率={args.hz}Hz")

    try:
        if args.mode == 'idle':
            run_idle(sock, args.hz)
        elif args.mode == 'static':
            run_static(sock, args.hz)
        elif args.mode == 'centering':
            run_centering(sock, args.hz)
        elif args.mode == 'drop':
            run_drop(sock, args.hz)
        elif args.mode == 'lost':
            run_lost(sock, args.hz)
    except KeyboardInterrupt:
        print("\n用户中断")
    except BrokenPipeError:
        print("连接断开")
    finally:
        sock.close()
        print("连接已关闭")


if __name__ == '__main__':
    main()
