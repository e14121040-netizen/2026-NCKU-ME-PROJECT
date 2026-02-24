#!/usr/bin/env python3
"""
=====================================================
 Chebyshev Lambda Linkage 六足步行機構 — 可視化模型
 2026 NCKU 機械專題實作
=====================================================

 功能：
 1. 單足 Chebyshev 四桿連桿動畫（顯示足端軌跡）
 2. 六足三角步態模擬（俯視 + 側視）
 3. 可調整連桿參數即時預覽

 使用方式：
   pip install matplotlib numpy
   python chebyshev_visualization.py

 快捷鍵：
   1 = 單足連桿動畫
   2 = 六足步態動畫
   3 = 參數比較圖
   q = 離開
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib import rcParams

# =====================================================
#  全局設定
# =====================================================
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')

# macOS 中文字型設定
rcParams['font.sans-serif'] = ['Arial Unicode MS', 'PingFang TC', 'Heiti TC',
                                'Apple LiGothic', 'STHeiti', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False  # 負號顯示修復
rcParams['font.size'] = 11
rcParams['figure.dpi'] = 100

# 預設色彩
COLORS = {
    'ground':  '#4A4A4A',   # 機架
    'crank':   '#E74C3C',   # 曲柄（紅）
    'coupler': '#3498DB',   # 耦桿（藍）
    'rocker':  '#2ECC71',   # 搖桿（綠）
    'trace':   '#F39C12',   # 足端軌跡（橙）
    'foot':    '#E74C3C',   # 足端點
    'support': '#2ECC71',   # 著地足（綠）
    'swing':   '#E74C3C',   # 擺動足（紅）
    'body':    '#34495E',   # 機身
    'bg':      '#F8F9FA',   # 背景
}


# =====================================================
#  Chebyshev Lambda Linkage 運動學
# =====================================================
class ChebyshevLinkage:
    """
    Chebyshev Lambda Linkage 四桿連桿機構

    固定支點：O1 (原點), O2 (ground, 0)
    曲柄：O1 → A (長度 = crank)
    耦桿：A → P (長度 = coupler)，P 為足端
    搖桿：O2 → B (長度 = rocker)
    耦桿約束：A → B 的距離也受限

    Chebyshev 經典 Lambda 機構：
    - O1 在左，O2 在右，間距 = ground
    - 曲柄從 O1 轉
    - 耦桿端點 P 在 coupler 桿上延伸（足端）
    """

    def __init__(self, crank=30, coupler=75, ground=75, rocker=75):
        self.crank = crank
        self.coupler = coupler
        self.ground = ground
        self.rocker = rocker

        # 固定支點
        self.O1 = np.array([0, 0])
        self.O2 = np.array([ground, 0])

    def solve(self, theta):
        """
        給定曲柄角度 theta（弧度），求各節點位置。
        回傳 (A, B, P) 座標 或 None（死點）
        """
        # 曲柄端點 A
        A = self.O1 + self.crank * np.array([np.cos(theta), np.sin(theta)])

        # 求搖桿端點 B：
        # |A - B| = coupler_AB (耦桿A到B的距離)
        # |O2 - B| = rocker
        # 這裡用 Chebyshev Lambda 的三角形耦桿：
        # A-B 的距離 = coupler（與搖桿等長）
        dx = A[0] - self.O2[0]
        dy = A[1] - self.O2[1]
        dist_AO2 = np.sqrt(dx**2 + dy**2)

        # 用餘弦定理求角度
        ab_len = self.coupler  # A-B 距離
        r = self.rocker

        cos_val = (dist_AO2**2 + r**2 - ab_len**2) / (2 * dist_AO2 * r)
        if abs(cos_val) > 1:
            return None

        angle_O2A = np.arctan2(dy, dx)
        angle_offset = np.arccos(cos_val)

        # 取下方解（足端朝下）
        angle_B = np.pi + angle_O2A + angle_offset if True else angle_O2A - angle_offset
        B = self.O2 + r * np.array([np.cos(angle_B), np.sin(angle_B)])

        # 足端 P：在耦桿 AB 的延長線上，P 在 B 的對側延伸
        # Lambda 機構中，P 是耦桿三角形的第三頂點
        # P 在 A-B 連線的中垂線延伸方向
        # 簡化：P = A + (A - B) 的方向延伸，使 |AP| = coupler
        # 更準確：P 是 AB 中點往下方延伸的點
        mid_AB = (A + B) / 2
        AB_vec = B - A
        AB_len = np.linalg.norm(AB_vec)

        if AB_len < 1e-10:
            return None

        # 垂直於 AB 的方向（向下）
        perp = np.array([AB_vec[1], -AB_vec[0]]) / AB_len

        # P 在 AB 中點下方，距離使得 |AP| = coupler
        # |AP|² = |AB/2|² + h²  →  h = sqrt(coupler² - (AB/2)²)
        half_AB = AB_len / 2
        h_sq = self.coupler**2 - half_AB**2
        if h_sq < 0:
            return None
        h = np.sqrt(h_sq)

        # 向下延伸（取 y 更小的方向）
        P1 = mid_AB + h * perp
        P2 = mid_AB - h * perp
        P = P1 if P1[1] < P2[1] else P2  # 取 y 值較小的（下方）

        return A, B, P

    def get_trajectory(self, n_points=360):
        """計算完整一圈的足端軌跡"""
        angles = np.linspace(0, 2 * np.pi, n_points)
        trajectory = []
        all_positions = []

        for theta in angles:
            result = self.solve(theta)
            if result is not None:
                A, B, P = result
                trajectory.append(P.copy())
                all_positions.append((theta, A.copy(), B.copy(), P.copy()))

        return np.array(trajectory), all_positions


# =====================================================
#  動畫 1：單足連桿機構動畫
# =====================================================
def animate_single_leg():
    """單足 Chebyshev 連桿動畫 + 軌跡"""
    linkage = ChebyshevLinkage(crank=30, coupler=75, ground=75, rocker=75)

    # 預先計算軌跡
    trajectory, all_positions = linkage.get_trajectory(360)

    fig, (ax_main, ax_info) = plt.subplots(1, 2, figsize=(14, 7),
                                            gridspec_kw={'width_ratios': [2, 1]})
    fig.patch.set_facecolor(COLORS['bg'])
    fig.suptitle('Chebyshev Lambda Linkage 單足動畫', fontsize=16, fontweight='bold')

    # 主動畫區
    ax_main.set_facecolor(COLORS['bg'])
    ax_main.set_xlim(-60, 140)
    ax_main.set_ylim(-120, 60)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.3)
    ax_main.set_xlabel('X (mm)')
    ax_main.set_ylabel('Y (mm)')
    ax_main.set_title('連桿機構側視圖')

    # 繪製元素
    trace_line, = ax_main.plot([], [], '-', color=COLORS['trace'], alpha=0.4, lw=2,
                                label='足端軌跡')
    # 完整軌跡（淡色背景）
    ax_main.plot(trajectory[:, 0], trajectory[:, 1], '--', color=COLORS['trace'],
                 alpha=0.15, lw=1)

    ground_line, = ax_main.plot([], [], 'o-', color=COLORS['ground'], lw=3,
                                 markersize=8, label='機架 (Ground)')
    crank_line, = ax_main.plot([], [], 'o-', color=COLORS['crank'], lw=3,
                                markersize=6, label='曲柄 (Crank)')
    coupler_AB, = ax_main.plot([], [], 'o-', color=COLORS['coupler'], lw=2.5,
                                markersize=5, label='耦桿 (Coupler)')
    rocker_line, = ax_main.plot([], [], 'o-', color=COLORS['rocker'], lw=3,
                                 markersize=6, label='搖桿 (Rocker)')
    coupler_AP, = ax_main.plot([], [], 'o-', color=COLORS['coupler'], lw=2.5,
                                markersize=5)
    coupler_BP, = ax_main.plot([], [], 'o-', color=COLORS['coupler'], lw=2.5,
                                markersize=5)

    foot_dot, = ax_main.plot([], [], 'o', color=COLORS['foot'], markersize=12,
                              zorder=5, label='足端 P')

    # 固定支點裝飾（三角形底座）
    for pos, name in [(linkage.O1, 'O1'), (linkage.O2, 'O2')]:
        tri = patches.RegularPolygon(pos, 3, radius=5, orientation=np.pi,
                                      facecolor=COLORS['ground'], alpha=0.6)
        ax_main.add_patch(tri)
        ax_main.annotate(name, pos, textcoords="offset points", xytext=(0, 10),
                         ha='center', fontsize=10, fontweight='bold')

    ax_main.legend(loc='upper right', fontsize=9)

    # 地面線
    ground_y = trajectory[:, 1].min() + 2
    ax_main.axhline(y=ground_y, color='#8B4513', linestyle='-', alpha=0.3, lw=2)
    ax_main.fill_between([-60, 140], ground_y - 20, ground_y,
                          color='#8B4513', alpha=0.08)

    # 資訊面板
    ax_info.set_facecolor(COLORS['bg'])
    ax_info.set_xlim(0, 10)
    ax_info.set_ylim(0, 10)
    ax_info.axis('off')
    ax_info.set_title('參數資訊', fontweight='bold')

    # 靜態文字
    info_texts = [
        (9.5, '── 連桿尺寸 ──'),
        (8.8, f'曲柄 (Crank)    = {linkage.crank} mm'),
        (8.2, f'耦桿 (Coupler) = {linkage.coupler} mm'),
        (7.6, f'機架 (Ground)  = {linkage.ground} mm'),
        (7.0, f'搖桿 (Rocker)   = {linkage.rocker} mm'),
        (6.0, '── 步行特性 ──'),
    ]
    for y, txt in info_texts:
        ax_info.text(0.5, y, txt, fontsize=10, family='monospace',
                     verticalalignment='center')

    # 動態文字
    angle_text = ax_info.text(0.5, 5.2, '', fontsize=10, family='monospace')
    foot_text = ax_info.text(0.5, 4.6, '', fontsize=10, family='monospace')
    stride_text = ax_info.text(0.5, 3.8, '', fontsize=10, family='monospace')
    height_text = ax_info.text(0.5, 3.2, '', fontsize=10, family='monospace')
    phase_text = ax_info.text(0.5, 2.4, '', fontsize=12, fontweight='bold',
                               color=COLORS['support'])

    # 計算步幅和步高
    stride_x = trajectory[:, 0].max() - trajectory[:, 0].min()
    stride_y = trajectory[:, 1].max() - trajectory[:, 1].min()
    stride_text.set_text(f'步距 (Stride)    ≈ {stride_x:.1f} mm')
    height_text.set_text(f'步高 (Height)  ≈ {stride_y:.1f} mm')

    # 繪製足端軌跡 X-Y 小圖
    ax_mini = fig.add_axes([0.58, 0.08, 0.35, 0.22])
    ax_mini.set_facecolor('#FAFAFA')
    ax_mini.plot(trajectory[:, 0], trajectory[:, 1], '-', color=COLORS['trace'], lw=2)
    ax_mini.set_title('足端 P 軌跡', fontsize=9)
    ax_mini.set_xlabel('X (mm)', fontsize=8)
    ax_mini.set_ylabel('Y (mm)', fontsize=8)
    ax_mini.set_aspect('equal')
    ax_mini.grid(True, alpha=0.3)
    mini_dot, = ax_mini.plot([], [], 'o', color=COLORS['foot'], markersize=8, zorder=5)

    # 標記行走段和擺動段
    mid_y = (trajectory[:, 1].max() + trajectory[:, 1].min()) / 2
    walk_mask = trajectory[:, 1] < mid_y
    if np.any(walk_mask):
        walk_pts = trajectory[walk_mask]
        ax_mini.plot(walk_pts[:, 0], walk_pts[:, 1], '-', color=COLORS['support'],
                     lw=4, alpha=0.3, label='著地段')
    swing_mask = ~walk_mask
    if np.any(swing_mask):
        swing_pts = trajectory[swing_mask]
        ax_mini.plot(swing_pts[:, 0], swing_pts[:, 1], '-', color=COLORS['swing'],
                     lw=4, alpha=0.3, label='擺動段')
    ax_mini.legend(fontsize=7, loc='upper left')

    trail_x, trail_y = [], []

    def update(frame):
        idx = frame % len(all_positions)
        theta, A, B, P = all_positions[idx]

        # 更新連桿
        ground_line.set_data([linkage.O1[0], linkage.O2[0]],
                              [linkage.O1[1], linkage.O2[1]])
        crank_line.set_data([linkage.O1[0], A[0]], [linkage.O1[1], A[1]])
        coupler_AB.set_data([A[0], B[0]], [A[1], B[1]])
        coupler_AP.set_data([A[0], P[0]], [A[1], P[1]])
        coupler_BP.set_data([B[0], P[0]], [B[1], P[1]])
        rocker_line.set_data([linkage.O2[0], B[0]], [linkage.O2[1], B[1]])
        foot_dot.set_data([P[0]], [P[1]])

        # 軌跡尾巴
        trail_x.append(P[0])
        trail_y.append(P[1])
        if len(trail_x) > 120:
            trail_x.pop(0)
            trail_y.pop(0)
        trace_line.set_data(trail_x, trail_y)

        # 小圖上的點
        mini_dot.set_data([P[0]], [P[1]])

        # 更新文字
        deg = np.degrees(theta)
        angle_text.set_text(f'曲柄角度 θ = {deg:.0f}°')
        foot_text.set_text(f'足端 P = ({P[0]:.1f}, {P[1]:.1f})')

        # 判斷著地/擺動相位
        if P[1] < mid_y:
            phase_text.set_text('▼ 著地行走中')
            phase_text.set_color(COLORS['support'])
        else:
            phase_text.set_text('▲ 擺動抬腳中')
            phase_text.set_color(COLORS['swing'])

        return (ground_line, crank_line, coupler_AB, coupler_AP, coupler_BP,
                rocker_line, foot_dot, trace_line, mini_dot,
                angle_text, foot_text, phase_text)

    anim = FuncAnimation(fig, update, frames=360, interval=30, blit=False)
    plt.tight_layout()
    plt.show()
    return anim


# =====================================================
#  動畫 2：六足三角步態動畫
# =====================================================
def animate_hexapod():
    """六足三角步態動畫（側視 + 俯視）"""
    linkage = ChebyshevLinkage(crank=30, coupler=75, ground=75, rocker=75)
    trajectory, all_positions = linkage.get_trajectory(360)
    n = len(all_positions)

    # 六足配置
    # 左側：足 1,2,3（馬達 A）  右側：足 4,5,6（馬達 B）
    # 三角步態：左側和右側曲柄相差 180°
    # 同側：足1,足3 同相(0°)，足2 反相(180°)
    leg_configs = [
        {'name': '足1(左前)', 'side': 'L', 'x_offset': -60, 'y_row': 60,  'phase': 0},
        {'name': '足2(左中)', 'side': 'L', 'x_offset': -60, 'y_row': 0,   'phase': 180},
        {'name': '足3(左後)', 'side': 'L', 'x_offset': -60, 'y_row': -60, 'phase': 0},
        {'name': '足4(右前)', 'side': 'R', 'x_offset': 60,  'y_row': 60,  'phase': 180},
        {'name': '足5(右中)', 'side': 'R', 'x_offset': 60,  'y_row': 0,   'phase': 0},
        {'name': '足6(右後)', 'side': 'R', 'x_offset': 60,  'y_row': -60, 'phase': 180},
    ]

    mid_y = (trajectory[:, 1].max() + trajectory[:, 1].min()) / 2

    fig = plt.figure(figsize=(15, 8))
    fig.patch.set_facecolor(COLORS['bg'])
    fig.suptitle('Chebyshev 六足三角步態 (Tripod Gait)', fontsize=16, fontweight='bold')

    # 左：側視圖（展示所有足的運動）
    ax_side = fig.add_subplot(121)
    ax_side.set_facecolor(COLORS['bg'])
    ax_side.set_xlim(-30, 130)
    ax_side.set_ylim(-130, 50)
    ax_side.set_aspect('equal')
    ax_side.grid(True, alpha=0.3)
    ax_side.set_xlabel('X (mm)')
    ax_side.set_ylabel('Y (mm)')
    ax_side.set_title('側視圖 — 連桿運動')

    # 地面
    ground_y = trajectory[:, 1].min() + 2
    ax_side.axhline(y=ground_y, color='#8B4513', linestyle='-', alpha=0.3, lw=2)
    ax_side.fill_between([-30, 130], ground_y - 15, ground_y,
                          color='#8B4513', alpha=0.08)

    # 軌跡背景
    ax_side.plot(trajectory[:, 0], trajectory[:, 1], '--', color=COLORS['trace'],
                 alpha=0.2, lw=1)

    # 固定支點
    for pos, name in [(linkage.O1, 'O1'), (linkage.O2, 'O2')]:
        tri = patches.RegularPolygon(pos, 3, radius=4, orientation=np.pi,
                                      facecolor=COLORS['ground'], alpha=0.6)
        ax_side.add_patch(tri)

    # 側視圖繪製元素（一組連桿用兩個相位展示）
    side_elements = {}
    for group_name, phase_offset, color_alpha in [('Group A (足1,3,5)', 0, 0.9),
                                                    ('Group B (足2,4,6)', 180, 0.5)]:
        grp = {}
        grp['crank'], = ax_side.plot([], [], 'o-', color=COLORS['crank'], lw=2.5,
                                      markersize=5, alpha=color_alpha)
        grp['coupler_AB'], = ax_side.plot([], [], 'o-', color=COLORS['coupler'], lw=2,
                                           markersize=4, alpha=color_alpha)
        grp['coupler_AP'], = ax_side.plot([], [], '-', color=COLORS['coupler'], lw=2,
                                           alpha=color_alpha)
        grp['coupler_BP'], = ax_side.plot([], [], '-', color=COLORS['coupler'], lw=2,
                                           alpha=color_alpha)
        grp['rocker'], = ax_side.plot([], [], 'o-', color=COLORS['rocker'], lw=2.5,
                                       markersize=5, alpha=color_alpha)
        grp['foot'], = ax_side.plot([], [], 'o', markersize=10, zorder=5,
                                     alpha=color_alpha)
        grp['phase_offset'] = phase_offset
        side_elements[group_name] = grp

    ax_side.plot([], [], 'o', color=COLORS['support'], markersize=8, label='著地 (支撐)')
    ax_side.plot([], [], 'o', color=COLORS['swing'], markersize=8, label='擺動 (抬腳)')
    ax_side.legend(loc='upper right', fontsize=9)

    # 右：俯視圖
    ax_top = fig.add_subplot(122)
    ax_top.set_facecolor(COLORS['bg'])
    ax_top.set_xlim(-120, 120)
    ax_top.set_ylim(-100, 100)
    ax_top.set_aspect('equal')
    ax_top.grid(True, alpha=0.2)
    ax_top.set_xlabel('左右 (mm)')
    ax_top.set_ylabel('前後 (mm)')
    ax_top.set_title('俯視圖 — 三角步態')

    # 繪製機身
    body_rect = patches.FancyBboxPatch((-35, -50), 70, 100, boxstyle="round,pad=5",
                                        facecolor=COLORS['body'], alpha=0.3)
    ax_top.add_patch(body_rect)
    ax_top.annotate('機身', (0, 0), ha='center', va='center',
                     fontsize=12, fontweight='bold', color=COLORS['body'])
    ax_top.annotate('▲ 前進方向', (0, 85), ha='center', fontsize=10,
                     color=COLORS['body'])

    # 六足俯視標記
    top_feet = {}
    top_legs = {}
    for lc in leg_configs:
        dot, = ax_top.plot([], [], 'o', markersize=14, zorder=5)
        top_feet[lc['name']] = dot
        line, = ax_top.plot([], [], '-', color=COLORS['body'], lw=2, alpha=0.5)
        top_legs[lc['name']] = line
        ax_top.annotate(lc['name'][:2], (lc['x_offset'], lc['y_row']),
                         ha='center', va='center', fontsize=7,
                         fontweight='bold', color='white', zorder=6)

    # 三角形連線
    tri_line_A, = ax_top.plot([], [], '--', color=COLORS['support'], lw=2, alpha=0.4)
    tri_line_B, = ax_top.plot([], [], '--', color=COLORS['swing'], lw=2, alpha=0.4)

    phase_label = ax_top.text(0, -90, '', ha='center', fontsize=11, fontweight='bold')

    def get_foot_state(frame_idx, phase_offset):
        """取得特定相位的足端位置和狀態"""
        idx = (frame_idx + int(phase_offset / 360 * n)) % n
        if idx < len(all_positions):
            _, A, B, P = all_positions[idx]
            is_ground = P[1] < mid_y  # 著地 or 擺動
            return A, B, P, is_ground
        return None, None, None, False

    def update(frame):
        # 更新側視圖 — 兩組連桿
        for group_name, grp in side_elements.items():
            A, B, P, is_ground = get_foot_state(frame, grp['phase_offset'])
            if A is None:
                continue

            grp['crank'].set_data([linkage.O1[0], A[0]], [linkage.O1[1], A[1]])
            grp['coupler_AB'].set_data([A[0], B[0]], [A[1], B[1]])
            grp['coupler_AP'].set_data([A[0], P[0]], [A[1], P[1]])
            grp['coupler_BP'].set_data([B[0], P[0]], [B[1], P[1]])
            grp['rocker'].set_data([linkage.O2[0], B[0]], [linkage.O2[1], B[1]])
            grp['foot'].set_data([P[0]], [P[1]])
            grp['foot'].set_color(COLORS['support'] if is_ground else COLORS['swing'])

        # 更新俯視圖
        support_pts = []
        swing_pts = []

        for lc in leg_configs:
            _, _, P, is_ground = get_foot_state(frame, lc['phase'])
            if P is None:
                continue

            dot = top_feet[lc['name']]
            line = top_legs[lc['name']]

            # 俯視圖中足端的位置：x 方向用 offset，y 方向用 row + 足端 x 的微小偏移
            foot_x = lc['x_offset'] + (15 if lc['side'] == 'R' else -15)
            foot_y = lc['y_row']

            dot.set_data([lc['x_offset']], [lc['y_row']])
            line.set_data([0 if lc['x_offset'] > 0 else 0, lc['x_offset']],
                          [lc['y_row'], lc['y_row']])

            if is_ground:
                dot.set_color(COLORS['support'])
                dot.set_markersize(16)
                support_pts.append((lc['x_offset'], lc['y_row']))
            else:
                dot.set_color(COLORS['swing'])
                dot.set_markersize(11)
                swing_pts.append((lc['x_offset'], lc['y_row']))

        # 畫三角形支撐線
        if len(support_pts) >= 3:
            pts = support_pts + [support_pts[0]]
            tri_line_A.set_data([p[0] for p in pts], [p[1] for p in pts])
        else:
            tri_line_A.set_data([], [])

        if len(swing_pts) >= 3:
            pts = swing_pts + [swing_pts[0]]
            tri_line_B.set_data([p[0] for p in pts], [p[1] for p in pts])
        else:
            tri_line_B.set_data([], [])

        # 相位標籤
        _, _, P_check, is_g = get_foot_state(frame, 0)
        if is_g:
            phase_label.set_text('Phase: 足1,5,3 著地支撐  |  足4,2,6 擺動')
            phase_label.set_color(COLORS['support'])
        else:
            phase_label.set_text('Phase: 足4,2,6 著地支撐  |  足1,5,3 擺動')
            phase_label.set_color(COLORS['swing'])

        return []

    anim = FuncAnimation(fig, update, frames=n, interval=30, blit=False)
    plt.tight_layout()
    plt.show()
    return anim


# =====================================================
#  靜態圖 3：參數比較
# =====================================================
def compare_parameters():
    """不同連桿參數的足端軌跡比較"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.patch.set_facecolor(COLORS['bg'])
    fig.suptitle('Chebyshev Linkage — 參數比較', fontsize=16, fontweight='bold')

    # 不同配置
    configs = [
        {'crank': 25, 'coupler': 62.5, 'ground': 62.5, 'rocker': 62.5,
         'title': '小尺寸 (1:2.5 經典比)'},
        {'crank': 30, 'coupler': 75, 'ground': 75, 'rocker': 75,
         'title': '★ 建議尺寸 (30/75/75/75)'},
        {'crank': 35, 'coupler': 87.5, 'ground': 87.5, 'rocker': 87.5,
         'title': '大尺寸 (1:2.5 經典比)'},
        {'crank': 30, 'coupler': 90, 'ground': 75, 'rocker': 75,
         'title': '長耦桿 (30/90/75/75)'},
    ]

    for ax, cfg in zip(axes.flat, configs):
        ax.set_facecolor(COLORS['bg'])

        linkage = ChebyshevLinkage(cfg['crank'], cfg['coupler'],
                                    cfg['ground'], cfg['rocker'])
        traj, _ = linkage.get_trajectory(500)

        if len(traj) == 0:
            ax.text(0.5, 0.5, '無法運動\n（死點）', transform=ax.transAxes,
                    ha='center', va='center', fontsize=14, color='red')
            ax.set_title(cfg['title'])
            continue

        # 繪製軌跡
        mid_y = (traj[:, 1].max() + traj[:, 1].min()) / 2
        walk_mask = traj[:, 1] < mid_y

        ax.plot(traj[:, 0], traj[:, 1], '-', color=COLORS['trace'], lw=2, alpha=0.6)

        if np.any(walk_mask):
            walk_pts = traj[walk_mask]
            ax.plot(walk_pts[:, 0], walk_pts[:, 1], '-', color=COLORS['support'],
                    lw=4, alpha=0.5, label='著地段')
        if np.any(~walk_mask):
            swing_pts = traj[~walk_mask]
            ax.plot(swing_pts[:, 0], swing_pts[:, 1], '-', color=COLORS['swing'],
                    lw=3, alpha=0.4, label='擺動段')

        # 繪製固定支點
        ax.plot(*linkage.O1, 's', color=COLORS['ground'], markersize=8)
        ax.plot(*linkage.O2, 's', color=COLORS['ground'], markersize=8)
        ax.plot([linkage.O1[0], linkage.O2[0]], [linkage.O1[1], linkage.O2[1]],
                '-', color=COLORS['ground'], lw=2)

        stride_x = traj[:, 0].max() - traj[:, 0].min()
        stride_y = traj[:, 1].max() - traj[:, 1].min()

        ax.set_title(cfg['title'], fontweight='bold',
                     color='#E74C3C' if '★' in cfg['title'] else 'black')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')

        info = f'步距≈{stride_x:.0f}mm  步高≈{stride_y:.0f}mm'
        ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        ax.legend(fontsize=8, loc='upper right')

    plt.tight_layout()
    plt.show()


# =====================================================
#  主選單
# =====================================================
def main():
    print("=" * 55)
    print("  Chebyshev Lambda Linkage 六足步行機構 可視化模型")
    print("  2026 NCKU 機械專題實作")
    print("=" * 55)
    print()
    print("  [1] 單足連桿動畫（含軌跡 + 參數資訊）")
    print("  [2] 六足三角步態動畫（側視 + 俯視）")
    print("  [3] 參數比較圖（不同連桿尺寸）")
    print("  [q] 離開")
    print()

    while True:
        choice = input("請選擇功能 (1/2/3/q): ").strip()

        if choice == '1':
            print("啟動單足連桿動畫...")
            animate_single_leg()
        elif choice == '2':
            print("啟動六足步態動畫...")
            animate_hexapod()
        elif choice == '3':
            print("產生參數比較圖...")
            compare_parameters()
        elif choice.lower() == 'q':
            print("再見！")
            break
        else:
            print("無效選擇，請輸入 1, 2, 3 或 q")


if __name__ == '__main__':
    main()
