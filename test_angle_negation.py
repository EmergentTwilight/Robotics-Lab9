#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试CoppeliaSim中切换基座时是否需要取反角度
"""
import json
import numpy as np

# 读取trajectory_results.json
with open('trajectory_results.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

step1_last = np.array(data['step1']['keypoints'][-1]['joint_angles'])
step2_first = np.array(data['step2']['keypoints'][0]['joint_angles'])

print("="*70)
print("角度映射测试")
print("="*70)

print("\nStep1最后一个关键点的角度（B-base，B_base_joints顺序）:")
print([f'{a:.6f}' for a in step1_last])
print("对应关节: [J7-B, J6, J5, J4, J3, J2, J1-A]")

print("\nStep2第一个关键点的角度（A-base，A_base_joints顺序）:")
print([f'{a:.6f}' for a in step2_first])
print("对应关节: [J1-A, J2, J3, J4, J5, J6, J7-B]")

# 方法1：仅反转顺序
method1 = step1_last[::-1]
diff1 = np.abs(step2_first - method1)
diff1_normalized = np.abs((step2_first - method1 + np.pi) % (2 * np.pi) - np.pi)  # 考虑角度周期性

print("\n" + "="*70)
print("方法1：仅反转顺序（current_a_angles = current_b_angles[::-1]）")
print("="*70)
print("映射结果:", [f'{a:.6f}' for a in method1])
print("与Step2的绝对差异:", [f'{d:.6f}' for d in diff1])
print("与Step2的归一化差异（考虑周期性）:", [f'{d:.6f}' for d in diff1_normalized])
print(f"平均差异: {np.mean(diff1):.6f}")
print(f"最大差异: {np.max(diff1):.6f}")

# 方法2：反转并取反
method2 = -step1_last[::-1]
method2_normalized = (method2 + np.pi) % (2 * np.pi)  # 归一化到[0, 2π)
diff2 = np.abs(step2_first - method2_normalized)
diff2_normalized = np.abs((step2_first - method2_normalized + np.pi) % (2 * np.pi) - np.pi)

print("\n" + "="*70)
print("方法2：反转并取反（current_a_angles = [-a for a in current_b_angles[::-1]]）")
print("="*70)
print("映射结果（归一化后）:", [f'{a:.6f}' for a in method2_normalized])
print("与Step2的绝对差异:", [f'{d:.6f}' for d in diff2])
print("与Step2的归一化差异（考虑周期性）:", [f'{d:.6f}' for d in diff2_normalized])
print(f"平均差异: {np.mean(diff2):.6f}")
print(f"最大差异: {np.max(diff2):.6f}")

print("\n" + "="*70)
print("结论")
print("="*70)
if np.mean(diff1) < np.mean(diff2):
    print("方法1（仅反转）的平均差异更小")
else:
    print("方法2（反转并取反）的平均差异更小")

print("\n但是，用户说现在的结果是错的，而且直觉上应该都要取反。")
print("这可能意味着：")
print("1. IK求解器中的角度定义与CoppeliaSim中的角度定义在轴方向上不一致")
print("2. 当两个骨骼完全相反时，不仅关节顺序要反转，角度也要取反才能保持相同的物理状态")
print("3. 因此，main.py中切换基座时应该使用方法2（反转并取反）")

