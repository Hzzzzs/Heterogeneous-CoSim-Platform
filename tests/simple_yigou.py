"""
Module: simple_yigou.py
Description: 基于全局拉普拉斯一致性的空地异构 (UAV+UGV) 统一编队算法
Author: Zishuo Huang
"""
import numpy as np

def get_heterogeneous_consensus_params():
    num_uavs = 3
    num_ugvs = 3
    total_agents = num_uavs + num_ugvs

    # 1. 统一初始位置矩阵 (前3个是UAV，后3个是UGV)
    init_pos = np.array([
        # --- UAV 空中三角形 ---
        [0.0,  1.5, 1.0],   
        [-1.5, -1.0, 1.0],  
        [1.5,  -1.0, 1.0],  
        # --- UGV 地面大三角形 ---
        [0.0,  3.0, 0.1],   
        [-3.0, -2.0, 0.1],  
        [3.0,  -2.0, 0.1]   
    ], dtype=float)

    init_pos = 0.5*init_pos

    # 2. 编队相对拓扑偏置 H (与初始位置重合，确保开局误差为0)
    H = np.copy(init_pos)

    # 3. 构造 6x6 的全局拉普拉斯矩阵 L
    # 为了让天地强协同，我们让这 6 个智能体在算法上“全连接”互相通信
    # 全连接图拉普拉斯矩阵的特征：对角线为 N-1 (即 5)，其余元素为 -1
    L = 6 * np.eye(total_agents) - np.ones((total_agents, total_agents))

    return num_uavs, num_ugvs, total_agents, init_pos, H, L