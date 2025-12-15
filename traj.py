# 该文件通过定义关键末端位置和欧拉角，通过IK计算出每个对应关键点的关节角度

import os
import sys
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# 添加IK_Solver路径
current_dir = os.path.dirname(os.path.abspath(__file__))
ik_solver_dir = os.path.join(current_dir, 'IK_Solver')
sys.path.append(ik_solver_dir)

from data_io import load_skeleton, initialize_tpose, euler_to_transform
from solver.solve_ik import solve_ik
from solver.ik_core import build_ik_chain, compute_error_vector
from model.joint import FixedJoint

# 调试标志
DEBUG = True

# step 1: based on B [0.35, 0, 0.2], [0, 0, 0]； J7-B为基座, J1-A为末端

POS_A_REL = [
    [0.3, 0, 0],
    [0.3, 0, 0.1],
    [-0.15, 0.3, 0.1],
    [-0.6, 0, 0.1],
    [-0.6, 0, 0],
]

EULER_A = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
]

JOINT_ANGLE_A = [
    [0, 0, 0, 0, 0, 0, 0],
    None,
    None,
    None,
    None,
]

JOINT_ANGLE_A_TOLERANCE=[
    1e-2,
    1,
    1,
    1,
    1e-4,
]

# ------------------------------------------------------------


# step 2: A [-0.25, 0, 0.2], [0, 0, 0]； J1-A为基座, J7-B为末端

POS_B_REL = [
    [0.6, 0, 0],
    [0.6, 0, 0.1],
    [0.1, 0.4, -0.2],
    [-0.4, 0.2, -0.1],
    [-0.4, 0.1, -0.1],
]

EULER_B = [
    [0, 0, 0],
    [0, 0, 0],
    [-45, 0, 0],
    [-90, 0, 0],
    [-90, 0, 0],
]

JOINT_ANGLE_B = [None, None, None, None, None]

JOINT_ANGLE_B_TOLERANCE=[
    1e-4,
    1,
    1,
    1,
    1e-2,
]

def find_effector(node):
    """查找末端执行器：寻找没有子节点的 FixedJoint"""
    if isinstance(node, FixedJoint) and len(node.children) == 0:
        return node
    for child in node.children:
        result = find_effector(child)
        if result is not None:
            return result
    return None

def solve_trajectory(type, pos_rel, euler, joint_angles, joint_tolerance, step_name, initial_joint_angles=None):
    """
    通用轨迹求解函数
    
    :param type: "A-base" 或 "B-base"，决定使用哪个骨骼文件
    :param pos_rel: 相对位置列表（米，相对于基座）
    :param euler: 欧拉角列表（度）
    :param joint_angles: 关节角度列表（会被修改，单位为弧度）
    :param joint_tolerance: 姿态容差列表（弧度）
    :param step_name: 步骤名称（用于打印）
    :param initial_joint_angles: 第0个关键点的初始关节角度（弧度），如果提供且joint_angles[0]为None，则使用此值
    """
    print("=" * 50)
    print(f"开始求解 {step_name}")
    print("=" * 50)
    
    # 加载骨骼
    skeleton_path = os.path.join(current_dir, 'data', f'{type}-arm.json')
    print(f"加载骨骼: {skeleton_path}")
    root, joint_map = load_skeleton(skeleton_path)
    
    # 查找末端执行器
    effector = find_effector(root)
    if effector is None:
        print("❌ 无法找到末端执行器")
        return
    print(f"末端执行器: {effector.name}")
    
    # 构建IK链
    ik_chain = build_ik_chain(root, effector)
    print(f"IK链构建成功，包含 {len(ik_chain)} 个关节")
    joint_names = [joint.name for joint in ik_chain]
    print(f"关节顺序: {joint_names}")
    
    # 初始化T-Pose
    initialize_tpose(root)
    root.update_global_transform()
    
    # 求解每个关键点
    num_keypoints = len(pos_rel)
    for i in range(num_keypoints):
        if joint_angles[i] is not None:
            print(f"\n--- 关键点 {i+1}/{num_keypoints} 已有值，跳过求解 ---") # 可通过第一个关键点设置初值
            for j, joint in enumerate(ik_chain):
                joint.q = joint_angles[i][j]  # joint_angles 已经是弧度
            root.update_global_transform()
            continue
        
        print(f"\n--- 求解关键点 {i+1}/{num_keypoints} ---")
        pos_m = np.array(pos_rel[i])
        euler_deg = np.array(euler[i])
        target_transform = euler_to_transform(pos_m, euler_deg)
        
        # 定义IK参数
        ik_params = {
            'max_iterations': 100,
            'position_tolerance': 1e-3,
            'orientation_tolerance': joint_tolerance[i],
            'damping': 0.0001,
            'enable_line_search': True
        }
        
        # 设置初始关节角度
        if i > 0:
            # 使用上一个关键点的值
            for j, joint in enumerate(ik_chain):
                joint.q = joint_angles[i-1][j]  # joint_angles 已经是弧度
            root.update_global_transform()
        elif initial_joint_angles is not None:
            # 第0个关键点且提供了初始值，使用初始值
            for j, joint in enumerate(ik_chain):
                joint.q = initial_joint_angles[j]  # initial_joint_angles 已经是弧度
            root.update_global_transform()
        # 如果 i==0 且没有提供 initial_joint_angles，则使用 T-Pose 的默认值
        
        # 求解IK前输出坐标和姿态（如果启用DEBUG）
        if DEBUG:
            root.update_global_transform()
            effector_pos_before = effector.global_transform[:3, 3]
            root_pos_before = root.global_transform[:3, 3]
            effector_rot_before = effector.global_transform[:3, :3]
            rot_before = R.from_matrix(effector_rot_before)
            euler_before = rot_before.as_euler('XYZ', degrees=True)
            quat_before = rot_before.as_quat()  # [x, y, z, w]格式
            print(f"  [DEBUG] IK求解前:")
            print(f"    末端执行器({effector.name})坐标: [{effector_pos_before[0]:.4f}, {effector_pos_before[1]:.4f}, {effector_pos_before[2]:.4f}]")
            print(f"    末端执行器({effector.name})姿态(欧拉角XYZ,度): [{euler_before[0]:.4f}, {euler_before[1]:.4f}, {euler_before[2]:.4f}]")
            print(f"    末端执行器({effector.name})姿态(四元数xyzw): [{quat_before[0]:.4f}, {quat_before[1]:.4f}, {quat_before[2]:.4f}, {quat_before[3]:.4f}]")
            print(f"    根节点({root.name})坐标: [{root_pos_before[0]:.4f}, {root_pos_before[1]:.4f}, {root_pos_before[2]:.4f}]")
            print(f"    关节角度: {[joint.q for joint in ik_chain]}")

        # 求解IK
        success = solve_ik(
            root=root,
            effector=effector,
            target_transform=target_transform,
            ik_chain=ik_chain,
            **ik_params
        )
        
        # 求解IK后输出坐标和姿态（如果启用DEBUG）
        if DEBUG:
            root.update_global_transform()
            effector_pos_after = effector.global_transform[:3, 3]
            root_pos_after = root.global_transform[:3, 3]
            effector_rot_after = effector.global_transform[:3, :3]
            rot_after = R.from_matrix(effector_rot_after)
            euler_after = rot_after.as_euler('XYZ', degrees=True)
            quat_after = rot_after.as_quat()  # [x, y, z, w]格式
            
            target_pos = target_transform[:3, 3]
            target_rot = target_transform[:3, :3]
            rot_target = R.from_matrix(target_rot)
            euler_target = rot_target.as_euler('XYZ', degrees=True)
            quat_target = rot_target.as_quat()  # [x, y, z, w]格式
            
            print(f"  [DEBUG] IK求解后:")
            print(f"    末端执行器({effector.name})坐标: [{effector_pos_after[0]:.4f}, {effector_pos_after[1]:.4f}, {effector_pos_after[2]:.4f}]")
            print(f"    末端执行器({effector.name})姿态(欧拉角XYZ,度): [{euler_after[0]:.4f}, {euler_after[1]:.4f}, {euler_after[2]:.4f}]")
            print(f"    末端执行器({effector.name})姿态(四元数xyzw): [{quat_after[0]:.4f}, {quat_after[1]:.4f}, {quat_after[2]:.4f}, {quat_after[3]:.4f}]")
            print(f"    根节点({root.name})坐标: [{root_pos_after[0]:.4f}, {root_pos_after[1]:.4f}, {root_pos_after[2]:.4f}]")
            print(f"    关节角度: {[joint.q for joint in ik_chain]}")
            print(f"    目标位置: [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]")
            print(f"    目标姿态(欧拉角XYZ,度): [{euler_target[0]:.4f}, {euler_target[1]:.4f}, {euler_target[2]:.4f}]")
            print(f"    目标姿态(四元数xyzw): [{quat_target[0]:.4f}, {quat_target[1]:.4f}, {quat_target[2]:.4f}, {quat_target[3]:.4f}]")
            current_transform = effector.global_transform
            delta_x = compute_error_vector(current_transform, target_transform)
            pos_error = np.linalg.norm(delta_x[:3])  # 位置误差（米）
            orient_error_rad = np.linalg.norm(delta_x[3:])  # 姿态误差（弧度，轴-角向量的模长）
            orient_error_deg = np.rad2deg(orient_error_rad)
            print(f"    位置误差: {pos_error:.6f} m")
            print(f"    姿态误差: {orient_error_deg:.4f}° ({orient_error_rad:.6f} rad)")
        
        joint_angles[i] = [joint.q for joint in ik_chain]  # 直接保存弧度值
        print(f"✅ 求解成功！" if success else "❌ 求解失败！")
    
    print("\n" + "=" * 50)
    print(f"{step_name} 求解完成！")
    print("=" * 50)
    print("\n关节角度结果:")
    for i, angles in enumerate(joint_angles):
        if angles is not None:
            print(f"关键点 {i+1}: {[f'{a:.4f}' for a in angles]}")
        else:
            print(f"关键点 {i+1}: None")
    
    # 返回关节角度结果
    return joint_angles


# 执行求解
if __name__ == "__main__":
    # 求解 Step 1
    result_a = solve_trajectory(
        type="B-base",
        pos_rel=POS_A_REL,
        euler=EULER_A,
        joint_angles=JOINT_ANGLE_A,
        joint_tolerance=JOINT_ANGLE_A_TOLERANCE,
        step_name="Step 1: B为基座，A为末端执行器"
    )
    print("\n")
    
    # 求解 Step 2
    # 将 Step1 最后一个关键点的关节角度反转并取反，作为 Step2 第0个关键点的初始值
    # 因为 B-base 和 A-base 的关节顺序是相反的，且由于链的方向相反，角度需要取反：
    # B-base: [J7-B, J6, J5, J4, J3, J2, J1-A]
    # A-base: [J1-A, J2, J3, J4, J5, J6, J7-B]
    # 注意：当两个骨骼完全相反时，不仅需要反转顺序，还需要取反每个角度
    initial_angles_for_step2 = None
    if result_a is not None and len(result_a) > 0 and result_a[-1] is not None:
        # 反转 Step1 最后一个关键点的关节角度，并取反每个角度
        initial_angles_for_step2 = [-a for a in result_a[-1][::-1]]
        print(f"\n使用 Step1 最后一个关键点的关节角度（反转并取反后）作为 Step2 的初始值:")
        print(f"  Step1 最后: {[f'{a:.4f}' for a in result_a[-1]]}")
        print(f"  Step2 初始: {[f'{a:.4f}' for a in initial_angles_for_step2]}")
    
    result_b = solve_trajectory(
        type="A-base",
        pos_rel=POS_B_REL,
        euler=EULER_B,
        joint_angles=JOINT_ANGLE_B,
        joint_tolerance=JOINT_ANGLE_B_TOLERANCE,
        step_name="Step 2: A为基座，B为末端执行器",
        initial_joint_angles=initial_angles_for_step2
    )
    
    # 准备 JSON 数据
    output_data = {
        "step1": {
            "description": "B为基座，A为末端执行器",
            "base_type": "B-base",
            "keypoints": []
        },
        "step2": {
            "description": "A为基座，B为末端执行器",
            "base_type": "A-base",
            "keypoints": []
        }
    }
    
    # 填充 Step 1 数据
    if result_a is not None:
        for i, angles in enumerate(result_a):
            if angles is not None:
                # 将角度限制在 [0, 2π) 范围内
                normalized_angles = [float(a % (2 * np.pi)) for a in angles]
                output_data["step1"]["keypoints"].append({
                    "keypoint_index": i,
                    "position_relative": POS_A_REL[i],
                    "euler_angles": EULER_A[i],
                    "joint_angles": normalized_angles
                })
    
    # 填充 Step 2 数据
    if result_b is not None:
        for i, angles in enumerate(result_b):
            if angles is not None:
                # 将角度限制在 [0, 2π) 范围内
                normalized_angles = [float(a % (2 * np.pi)) for a in angles]
                output_data["step2"]["keypoints"].append({
                    "keypoint_index": i,
                    "position_relative": POS_B_REL[i],
                    "euler_angles": EULER_B[i],
                    "joint_angles": normalized_angles
                })
    
    # 保存为 JSON 文件
    output_path = os.path.join(current_dir, "trajectory_results.json")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print("\n" + "=" * 50)
    print(f"✅ 结果已保存到: {output_path}")
    print("=" * 50)