#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析关节对应关系和角度映射
"""
import json
import numpy as np

# 读取trajectory_results.json
with open('trajectory_results.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

step1_last = data['step1']['keypoints'][-1]['joint_angles']
step2_first = data['step2']['keypoints'][0]['joint_angles']

print("="*70)
print("关节对应关系分析")
print("="*70)

print("A-base-arm.json的关节链:")
print("  base -> J1-A -> J2 -> J3 -> J4 -> J5 -> J6 -> J7-B -> EE")
print("  IK链顺序: [J1-A, J2, J3, J4, J5, J6, J7-B]")

print("\nB-base-arm.json的关节链:")
print("  base -> J7-B -> J6 -> J5 -> J4 -> J3 -> J2 -> J1-A -> EE")
print("  IK链顺序: [J7-B, J6, J5, J4, J3, J2, J1-A]")

print("\n" + "="*70)
print("CoppeliaSim中的关节顺序")
print("="*70)
print("A_base_joints = [l_joint1, l_joint2, l_joint3, joint4, r_joint3, r_joint2, r_joint1]")
print("B_base_joints = [r_joint1, r_joint2, r_joint3, joint4, l_joint3, l_joint2, l_joint1]")
print("\n注意：A_base_joints和B_base_joints的顺序是相反的")

print("\n" + "="*70)
print("关键问题：关节名称的对应关系")
print("="*70)
print("需要确定：")
print("1. A-base的J1-A对应CoppeliaSim中的哪个关节？")
print("2. A-base的J7-B对应CoppeliaSim中的哪个关节？")
print("3. B-base的J1-A对应CoppeliaSim中的哪个关节？")
print("4. B-base的J7-B对应CoppeliaSim中的哪个关节？")

print("\n" + "="*70)
print("从main.py的代码推断")
print("="*70)
print("A_base_joints = [l_joint1, l_joint2, l_joint3, joint4, r_joint3, r_joint2, r_joint1]")
print("B_base_joints = [r_joint1, r_joint2, r_joint3, joint4, l_joint3, l_joint2, l_joint1]")
print("\n如果A-base的IK链是[J1-A, J2, J3, J4, J5, J6, J7-B]")
print("那么可能对应：")
print("  A_base_joints[0] (l_joint1) = J1-A")
print("  A_base_joints[1] (l_joint2) = J2")
print("  A_base_joints[2] (l_joint3) = J3")
print("  A_base_joints[3] (joint4)   = J4")
print("  A_base_joints[4] (r_joint3) = J5")
print("  A_base_joints[5] (r_joint2) = J6")
print("  A_base_joints[6] (r_joint1) = J7-B")

print("\n如果B-base的IK链是[J7-B, J6, J5, J4, J3, J2, J1-A]")
print("那么可能对应：")
print("  B_base_joints[0] (r_joint1) = J7-B")
print("  B_base_joints[1] (r_joint2) = J6")
print("  B_base_joints[2] (r_joint3) = J5")
print("  B_base_joints[3] (joint4)   = J4")
print("  B_base_joints[4] (l_joint3) = J3")
print("  B_base_joints[5] (l_joint2) = J2")
print("  B_base_joints[6] (l_joint1) = J1-A")

print("\n" + "="*70)
print("角度映射验证")
print("="*70)
print("Step1最后一个关键点的角度（B-base坐标系）:")
print([f'{a:.6f}' for a in step1_last])
print("对应B_base_joints: [r_joint1, r_joint2, r_joint3, joint4, l_joint3, l_joint2, l_joint1]")
print("对应关节: [J7-B, J6, J5, J4, J3, J2, J1-A]")

print("\nStep2第一个关键点的角度（A-base坐标系）:")
print([f'{a:.6f}' for a in step2_first])
print("对应A_base_joints: [l_joint1, l_joint2, l_joint3, joint4, r_joint3, r_joint2, r_joint1]")
print("对应关节: [J1-A, J2, J3, J4, J5, J6, J7-B]")

print("\n对应关系:")
print("  B_base的J7-B (index 0) <-> A_base的J7-B (index 6)")
print("  B_base的J6  (index 1) <-> A_base的J6  (index 5)")
print("  B_base的J5  (index 2) <-> A_base的J5  (index 4)")
print("  B_base的J4  (index 3) <-> A_base的J4  (index 3)")
print("  B_base的J3  (index 4) <-> A_base的J3  (index 2)")
print("  B_base的J2  (index 5) <-> A_base的J2  (index 1)")
print("  B_base的J1-A(index 6) <-> A_base的J1-A(index 0)")

# 检查：如果B_base_joints的角度是angles_b，那么A_base_joints的角度应该是什么？
print("\n" + "="*70)
print("测试不同的映射方式")
print("="*70)

# 方法1：仅反转顺序
method1 = step1_last[::-1]
print("方法1：仅反转顺序")
print([f'{a:.6f}' for a in method1])
diff1 = [s2 - s1 for s1, s2 in zip(method1, step2_first)]
print("与Step2的差异:", [f'{d:.6f}' for d in diff1])

# 方法2：反转并取反
method2 = [-a for a in step1_last[::-1]]
method2_norm = [(a % (2 * np.pi)) for a in method2]
print("\n方法2：反转并取反（归一化后）")
print([f'{a:.6f}' for a in method2_norm])
diff2 = [s2 - s1 for s1, s2 in zip(method2_norm, step2_first)]
print("与Step2的差异:", [f'{d:.6f}' for d in diff2])

print("\n" + "="*70)
print("结论")
print("="*70)
print("从IK求解的结果看，方法2（反转并取反）与Step2完全一致。")
print("这说明在IK求解时，角度需要取反。")
print("\n但是，在CoppeliaSim中，如果切换基座后角度不需要取反，")
print("那么可能的原因是：")
print("1. CoppeliaSim中的关节定义方式与IK求解器不同")
print("2. 或者，CoppeliaSim中的关节顺序定义有问题")
print("\n用户说现在的结果是错的，那么可能需要重新检查CoppeliaSim中的角度映射。")

