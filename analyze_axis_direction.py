#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析A-base和B-base中对应关节的轴方向
"""
import json

# 读取JSON文件
with open('data/A-base-arm.json', 'r', encoding='utf-8') as f:
    a_base = json.load(f)

with open('data/B-base-arm.json', 'r', encoding='utf-8') as f:
    b_base = json.load(f)

# 建立关节映射
a_joints = {}
for joint in a_base['joints']:
    if joint['type'] == 'revolute':
        a_joints[joint['name']] = {
            'axis': joint['axis'],
            'offset': joint['offset'],
            'parent': joint['parent']
        }

b_joints = {}
for joint in b_base['joints']:
    if joint['type'] == 'revolute':
        b_joints[joint['name']] = {
            'axis': joint['axis'],
            'offset': joint['offset'],
            'parent': joint['parent']
        }

print("="*70)
print("关节轴方向对比")
print("="*70)

# A-base的关节链顺序
a_chain = ['J1-A', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7-B']
# B-base的关节链顺序（相反）
b_chain = ['J7-B', 'J6', 'J5', 'J4', 'J3', 'J2', 'J1-A']

print("\nA-base链: ", a_chain)
print("B-base链: ", b_chain)

print("\n" + "="*70)
print("对应关节的轴方向对比")
print("="*70)

for i, joint_name in enumerate(a_chain):
    a_idx = i
    b_idx = len(a_chain) - 1 - i  # 对应的B-base索引
    
    a_joint = a_joints[joint_name]
    b_joint = b_joints[joint_name]
    
    a_axis = a_joint['axis']
    b_axis = b_joint['axis']
    
    # 检查轴是否相同或相反
    axis_same = all(abs(a - b) < 1e-6 for a, b in zip(a_axis, b_axis))
    axis_opposite = all(abs(a + b) < 1e-6 for a, b in zip(a_axis, b_axis))
    
    print(f"\n关节 {joint_name}:")
    print(f"  A-base轴: {a_axis}")
    print(f"  B-base轴: {b_axis}")
    print(f"  A-base offset: {a_joint['offset']}")
    print(f"  B-base offset: {b_joint['offset']}")
    
    if axis_same:
        print(f"  → 轴方向相同")
    elif axis_opposite:
        print(f"  → 轴方向相反（需要取反角度）")
    else:
        print(f"  → 轴方向不同（需要特殊处理）")

print("\n" + "="*70)
print("关键观察")
print("="*70)
print("如果两个骨骼完全相反，那么：")
print("1. 关节顺序相反")
print("2. offset方向相反（在某些轴上）")
print("3. 轴方向可能相反（在某些轴上）")
print("\n如果轴方向相反，那么角度需要取反才能保持相同的物理旋转效果。")
print("如果轴方向相同，那么角度不需要取反。")

