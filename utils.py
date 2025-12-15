import math
import numpy as np
from typing import Union



def angle_interpolate_2(start, end, t_start, t_end, t):
    """
    Interpolate angle between start and end angles at time t using quintic polynomial
    Args:
        start: start angle
        end: end angle
        t_start: start time
        t_end: end time
        t: current time

    Returns:
        interpolated angle
    """
    # Handle boundary conditions
    if t <= t_start:
        return start
    if t >= t_end:
        return end

    # Calculate time duration
    T = t_end - t_start
    if T <= 0:
        return start

    # Normalize time to [0, T]
    tau = t - t_start

    # Find shortest path for angle interpolation
    angle_diff = end - start
    if angle_diff > math.pi:
        end = end - 2 * math.pi
    elif angle_diff < -math.pi:
        end = end + 2 * math.pi

    # Calculate quintic polynomial coefficients
    # Using zero velocity and acceleration at boundaries for smooth motion
    a0 = start
    a1 = 0  # v_start = 0
    a2 = 0  # a_start = 0

    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    a3 = (20 * (end - start) - 8 * 0 * T - 12 * 0 * T) / (2 * T3)  # v_end = 0, a_end = 0
    a4 = (30 * (start - end) + 14 * 0 * T + 16 * 0 * T) / (2 * T4)
    a5 = (12 * (end - start) - 6 * 0 * T - 6 * 0 * T) / (2 * T5)

    # Evaluate quintic polynomial
    angle = (
        a0
        + a1 * tau
        + a2 * tau * tau
        + a3 * tau * tau * tau
        + a4 * tau * tau * tau * tau
        + a5 * tau * tau * tau * tau * tau
    )

    # Normalize angle to [-pi, pi]
    angle = math.remainder(angle, 2 * math.pi)

    return angle


def angle_interpolate_3(start, mid, end, t_start, t_mid, t_end, t):
    """
    Interpolate angle through start, mid, and end points at time t using quintic polynomials
    Args:
        start: start angle
        mid: middle angle
        end: end angle
        t_start: start time
        t_mid: middle time
        t_end: end time
        t: current time

    Returns:
        interpolated angle
    """
    # Handle boundary conditions
    if t <= t_start:
        return start
    if t >= t_end:
        return end

    # Calculate time durations
    T1 = t_mid - t_start
    T2 = t_end - t_mid

    if T1 <= 0 or T2 <= 0:
        return start

    # Process angles for continuity
    # Find shortest path for angle interpolation
    mid_processed = mid
    end_processed = end

    # Process mid angle
    diff_mid = mid - start
    if diff_mid > math.pi:
        mid_processed = mid - 2 * math.pi
    elif diff_mid < -math.pi:
        mid_processed = mid + 2 * math.pi

    # Process end angle relative to mid
    diff_end = end - mid_processed
    if diff_end > math.pi:
        end_processed = end - 2 * math.pi
    elif diff_end < -math.pi:
        end_processed = end + 2 * math.pi

    # First segment: start to mid
    if t <= t_mid:
        tau = t - t_start

        # Calculate quintic polynomial coefficients for first segment
        a0 = start
        a1 = 0  # v_start = 0
        a2 = 0  # a_start = 0

        T1_2 = T1 * T1
        T1_3 = T1_2 * T1
        T1_4 = T1_3 * T1
        T1_5 = T1_4 * T1

        # Estimate velocity at mid point
        v_mid = 0.5 * ((mid_processed - start) / T1 + (end_processed - mid_processed) / T2)

        a3 = (20 * (mid_processed - start) - (8 * v_mid + 12 * 0) * T1) / (2 * T1_3)
        a4 = (30 * (start - mid_processed) + (14 * v_mid + 16 * 0) * T1) / (2 * T1_4)
        a5 = (12 * (mid_processed - start) - (6 * v_mid + 6 * 0) * T1) / (2 * T1_5)

        # Evaluate quintic polynomial
        angle = (
            a0
            + a1 * tau
            + a2 * tau * tau
            + a3 * tau * tau * tau
            + a4 * tau * tau * tau * tau
            + a5 * tau * tau * tau * tau * tau
        )

    # Second segment: mid to end
    else:
        tau = t - t_mid

        # Calculate quintic polynomial coefficients for second segment
        a0 = mid_processed
        # Estimate velocity at mid point
        v_mid = 0.5 * ((mid_processed - start) / T1 + (end_processed - mid_processed) / T2)
        a1 = v_mid
        # Estimate acceleration at mid point
        a_mid = ((end_processed - mid_processed) / T2 - (mid_processed - start) / T1) / (T1 + T2) * 2
        a2 = a_mid / 2

        T2_2 = T2 * T2
        T2_3 = T2_2 * T2
        T2_4 = T2_3 * T2
        T2_5 = T2_4 * T2

        a3 = (20 * (end_processed - mid_processed) - (8 * 0 + 12 * v_mid) * T2 - (3 * a_mid - 0) * T2_2) / (2 * T2_3)
        a4 = (30 * (mid_processed - end_processed) + (14 * 0 + 16 * v_mid) * T2 + (3 * a_mid - 2 * 0) * T2_2) / (
            2 * T2_4
        )
        a5 = (12 * (end_processed - mid_processed) - (6 * 0 + 6 * v_mid) * T2 - (0 - a_mid) * T2_2) / (2 * T2_5)

        # Evaluate quintic polynomial
        angle = (
            a0
            + a1 * tau
            + a2 * tau * tau
            + a3 * tau * tau * tau
            + a4 * tau * tau * tau * tau
            + a5 * tau * tau * tau * tau * tau
        )

    # Normalize angle to [-pi, pi]
    angle = math.remainder(angle, 2 * math.pi)

    return angle

def switch_base(sim, chain, old_joints, new_joints):
    """
    通用的换基函数，保持新基的位置在换基前后不变
    
    Args:
        sim: CoppeliaSim仿真对象
        chain: 新的骨骼链（用于重新建立父子关系），chain[0]是新基
        old_joints: 旧基座下的关节列表（按照旧基座的顺序）
        new_joints: 新基座下的关节列表（按照新基座的顺序）
    
    Note:
        setObjectParent的第三个参数inplace=True表示保持对象在原位置。
        如果在非关节角全0的背景下使用该功能，可能会导致机械臂的"初态"发生变化。
        因此，在切换基座前需要先将所有关节角度设为0。
    """
    # 记录新基的位置（chain[0]就是新基）
    new_base = chain[0]
    new_base_position_before = sim.getObjectPosition(new_base, sim.handle_world)
    
    # 读取旧基座下的关节角度
    current_old_angles = [sim.getJointPosition(joint) for joint in old_joints]
    
    # 清零旧关节角度（避免inplace=True在非零角度下的问题）
    for joint in old_joints:
        sim.setJointPosition(joint, 0.0)
    
    # 切换基座：断开所有对象的父子关系
    for obj in chain:
        sim.setObjectParent(obj, -1, True)
    
    # 重新建立父子关系（使用inplace=True保持位置）
    root = par = chain[0]
    for obj in chain[1:]:
        sim.setObjectParent(obj, par, True)
        par = obj
    
    # 映射角度：反转并取反
    current_new_angles = [-a for a in current_old_angles[::-1]]
    
    # 设置新基座下的关节角度
    for joint_idx, joint in enumerate(new_joints):
        sim.setJointPosition(joint, current_new_angles[joint_idx])
    
    # 恢复新基位置（确保换基前后位置不变）
    sim.setObjectPosition(new_base, new_base_position_before, sim.handle_world)


def quaternion_to_rotation_matrix(quaternion: Union[np.ndarray, list, tuple]) -> np.ndarray:
    """
    将四元数转换为旋转矩阵
    
    :param quaternion: 四元数，格式为 [w, x, y, z] 或 (w, x, y, z)
    :return: 3x3 旋转矩阵
    """
    quaternion = np.asarray(quaternion, dtype=np.float64)
    
    if quaternion.shape != (4,):
        raise ValueError(f"Quaternion must be a 4-element array, got shape {quaternion.shape}")
    
    # 归一化四元数
    norm = np.linalg.norm(quaternion)
    if norm < 1e-10:
        raise ValueError(f"Quaternion norm too small: {norm}, cannot normalize")
    quaternion = quaternion / norm
    
    w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    
    R = np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y)],
        [    2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x)],
        [    2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ], dtype=np.float64)
    
    return R
