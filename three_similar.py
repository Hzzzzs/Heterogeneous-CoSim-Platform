import numpy as np
from numpy.linalg import eig, det

def get_complex_laplacian_formation():
    """
    计算编队控制矩阵 M 和初始状态 x0
    返回: M (矩阵), x0 (初始位置向量), n (无人机数量)
    """
    n = 4      
    d = 3      

    # 基础拉普拉斯矩阵定义
    L = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 1, -3 - np.sqrt(2), 1 + np.sqrt(2)],
        [1, 1, 1, -3]
    ], dtype=float)

    # 定义旋转角度 theta
    theta31, theta32, theta34 = 5 * np.pi / 4, 0, 5 * np.pi / 4
    theta41, theta42, theta43 = 0, 0, np.pi / 4

    # 构造复数拉普拉斯矩阵 Lxy
    Lxy = np.zeros((n, n), dtype=complex)
    Lxy[2, 0] = L[2, 0] * np.exp(1j * theta31)
    Lxy[2, 1] = L[2, 1] * np.exp(1j * theta32)
    Lxy[2, 3] = L[2, 3] * np.exp(1j * theta34)
    Lxy[2, 2] = -(Lxy[2, 0] + Lxy[2, 1] + Lxy[2, 3])
    
    Lxy[3, 0] = L[3, 0] * np.exp(1j * theta41)
    Lxy[3, 1] = L[3, 1] * np.exp(1j * theta42)
    Lxy[3, 2] = L[3, 2] * np.exp(1j * theta43)
    Lxy[3, 3] = -(Lxy[3, 0] + Lxy[3, 1] + Lxy[3, 2])

    # 稳定性增益 e 的计算
    A = Lxy[2:4, 2:4]
    m_dim = 2
    e = np.zeros(m_dim, dtype=complex)
    e[0] = 1.0 / det(A[0:1, 0:1])
    lambda_vals = eig(np.diag([e[0]]) @ A[0:1, 0:1])[0]
    e_diag = np.array([e[0]], dtype=complex)
    epsilon = 0.1
    for i in range(1, m_dim):
        e_prod = np.prod(e[:i])
        lambda_prod = np.prod(lambda_vals[:i])
        val = e_prod * det(A[:i+1, :i+1]) / lambda_prod
        e[i] = epsilon * np.exp(-1j * np.angle(val))
        e_diag = np.append(e_diag, e[i])
        lambda_vals = eig(np.diag(e_diag) @ A[:i+1, :i+1])[0]
    
    e = -e 

    def R_from_angle(phi):
        return np.array([
            [np.cos(phi), -np.sin(phi), 0],
            [np.sin(phi),  np.cos(phi), 0],
            [0,           0,           1]
        ])

    R1 = R_from_angle(np.angle(e[0]))
    R2 = R_from_angle(np.angle(e[1]))
    
    # 构造增益矩阵 D
    D = np.zeros((12, 12), dtype=float)
    D[:6, :6] = 0.1 * np.eye(6)
    D[6:9, 6:9] = np.abs(e[0]) * R1
    D[9:12, 9:12] = np.abs(e[1]) * R2

    # 构造整体控制矩阵 LL
    R31, R32, R34 = R_from_angle(theta31), R_from_angle(theta32), R_from_angle(theta34)
    R41, R42, R43 = R_from_angle(theta41), R_from_angle(theta42), R_from_angle(theta43)

    LL = np.zeros((12, 12), dtype=float)
    LL[6:9, 0:3] = L[2, 0] * R31
    LL[6:9, 3:6] = L[2, 1] * R32
    LL[6:9, 6:9] = -(L[2, 0] * R31 + L[2, 1] * R32 + L[2, 3] * R34)
    LL[6:9, 9:12] = L[2, 3] * R34
    
    LL[9:12, 0:3] = L[3, 0] * R41
    LL[9:12, 3:6] = L[3, 1] * R42
    LL[9:12, 6:9] = L[3, 2] * R43
    LL[9:12, 9:12] = -(L[3, 0] * R41 + L[3, 1] * R42 + L[3, 2] * R43)

    # 最终系统控制矩阵
    M = D @ LL  

    # 初始位置计算 (Leader 锚定与 Follower 随机化)
    scaling = 1.0
    thetaleader = 0.0
    Rleader = np.array([
        [np.cos(thetaleader),  np.sin(thetaleader), 0],
        [-np.sin(thetaleader), np.cos(thetaleader), 0],
        [0,                    0,                   1]
    ])
    trans = np.array([0, 0, 0]) 
    
    x0 = np.random.rand(n * d) 
    x0[0:3] = scaling * (Rleader @ np.array([0, 0, 3])) + trans
    x0[3:6] = scaling * (Rleader @ np.array([1, 0, 1])) + trans
    
    # 地面避障安全高度检查
    if x0[8] < 0.2: x0[8] = 0.5
    if x0[11] < 0.2: x0[11] = 0.5

    return M, x0, n