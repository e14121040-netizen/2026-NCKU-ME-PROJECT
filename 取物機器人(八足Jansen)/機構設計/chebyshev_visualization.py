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
from matplotlib.collections import LineCollection
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
rcParams['axes.unicode_minus'] = False
rcParams['font.size'] = 11
rcParams['figure.dpi'] = 120

# =====================================================
#  深色主題配色
# =====================================================
COLORS = {
    'bg_dark':    '#0d1117',     # 深色背景
    'bg_panel':   '#161b22',     # 面板背景
    'bg_axes':    '#0d1117',     # 軸背景
    'grid':       '#21262d',     # 格線
    'crank':      '#ff6b6b',     # 曲柄 — 鮮紅
    'coupler':    '#4ecdc4',     # 耦桿 — 青綠
    'rocker':     '#45b7d1',     # 搖桿 — 天藍
    'ground':     '#8b949e',     # 機架 — 銀灰
    'trace':      '#ffd93d',     # 軌跡 — 金黃
    'trace_dim':  '#b8860b',     # 軌跡淡色
    'foot':       '#ff6b6b',     # 足端
    'support':    '#56d364',     # 著地 — 霓虹綠
    'swing':      '#f97583',     # 擺動 — 亮粉紅
    'body':       '#6e7681',     # 機身
    'text':       '#e6edf3',     # 文字
    'text_dim':   '#8b949e',     # 次要文字
    'accent':     '#bc8cff',     # 強調紫
    'highlight':  '#ffa657',     # 高亮橘
    'glow_green': '#56d364',     # 綠色光暈
    'glow_red':   '#f97583',     # 紅色光暈
    'ground_fill':'#3d2b1f',     # 地面填充
    'ground_line':'#8b6914',     # 地面線
}


def apply_dark_style(ax, title=''):
    """統一套用深色主題到 axes"""
    ax.set_facecolor(COLORS['bg_axes'])
    ax.tick_params(colors=COLORS['text_dim'], labelsize=9)
    ax.xaxis.label.set_color(COLORS['text_dim'])
    ax.yaxis.label.set_color(COLORS['text_dim'])
    for spine in ax.spines.values():
        spine.set_color(COLORS['grid'])
        spine.set_linewidth(0.5)
    if title:
        ax.set_title(title, color=COLORS['text'], fontweight='bold', fontsize=12, pad=10)


def draw_glow_point(ax, x, y, color, size=12, glow_size=24, glow_alpha=0.25, zorder=10):
    """繪製帶光暈的關節點"""
    ax.plot(x, y, 'o', color=color, markersize=glow_size, alpha=glow_alpha, zorder=zorder - 1)
    ax.plot(x, y, 'o', color=color, markersize=size, alpha=0.9, zorder=zorder)
    return ax.plot(x, y, 'o', color='white', markersize=size * 0.35, alpha=0.7, zorder=zorder + 1)


def draw_fixed_pivot(ax, pos, name, size=7):
    """繪製帶底座的固定支點"""
    # 底座三角形
    tri = patches.RegularPolygon(pos, 3, radius=size, orientation=np.pi,
                                  facecolor=COLORS['ground'], alpha=0.4,
                                  edgecolor=COLORS['ground'], linewidth=1.5)
    ax.add_patch(tri)
    # 光暈
    ax.plot(pos[0], pos[1], 'o', color=COLORS['ground'], markersize=18, alpha=0.15)
    ax.plot(pos[0], pos[1], 'o', color=COLORS['ground'], markersize=10, alpha=0.6)
    ax.annotate(name, pos, textcoords="offset points", xytext=(0, 13),
                ha='center', fontsize=10, fontweight='bold',
                color=COLORS['text'], alpha=0.8)


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
        A = self.O1 + self.crank * np.array([np.cos(theta), np.sin(theta)])

        dx = A[0] - self.O2[0]
        dy = A[1] - self.O2[1]
        dist_AO2 = np.sqrt(dx**2 + dy**2)

        ab_len = self.coupler
        r = self.rocker

        cos_val = (dist_AO2**2 + r**2 - ab_len**2) / (2 * dist_AO2 * r)
        if abs(cos_val) > 1:
            return None

        angle_O2A = np.arctan2(dy, dx)
        angle_offset = np.arccos(cos_val)

        angle_B = np.pi + angle_O2A + angle_offset
        B = self.O2 + r * np.array([np.cos(angle_B), np.sin(angle_B)])

        mid_AB = (A + B) / 2
        AB_vec = B - A
        AB_len = np.linalg.norm(AB_vec)

        if AB_len < 1e-10:
            return None

        perp = np.array([AB_vec[1], -AB_vec[0]]) / AB_len

        half_AB = AB_len / 2
        h_sq = self.coupler**2 - half_AB**2
        if h_sq < 0:
            return None
        h = np.sqrt(h_sq)

        P1 = mid_AB + h * perp
        P2 = mid_AB - h * perp
        P = P1 if P1[1] < P2[1] else P2

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
    trajectory, all_positions = linkage.get_trajectory(360)

    fig, (ax_main, ax_info) = plt.subplots(1, 2, figsize=(15, 8),
                                            gridspec_kw={'width_ratios': [2, 1]})
    fig.patch.set_facecolor(COLORS['bg_dark'])
    fig.suptitle('Chebyshev Lambda Linkage — 單足連桿動畫',
                 fontsize=17, fontweight='bold', color=COLORS['text'], y=0.97)

    # === 主動畫區 ===
    apply_dark_style(ax_main, '連桿機構側視圖')
    ax_main.set_xlim(-65, 145)
    ax_main.set_ylim(-125, 65)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.15, color=COLORS['grid'], linewidth=0.5)
    ax_main.set_xlabel('X (mm)', fontsize=10)
    ax_main.set_ylabel('Y (mm)', fontsize=10)

    # 完整軌跡（背景 — 漸層金黃）
    n_traj = len(trajectory)
    points = trajectory.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    alphas = np.linspace(0.08, 0.3, len(segments))
    colors_traj = np.zeros((len(segments), 4))
    base_rgb = np.array([1.0, 0.85, 0.24])  # gold
    for i in range(len(segments)):
        colors_traj[i, :3] = base_rgb
        colors_traj[i, 3] = alphas[i]
    lc_bg = LineCollection(segments, colors=colors_traj, linewidths=2.5)
    ax_main.add_collection(lc_bg)

    # 動態軌跡尾巴
    trace_line, = ax_main.plot([], [], '-', color=COLORS['trace'], alpha=0.7, lw=3,
                                label='足端軌跡', zorder=3)

    # 連桿繪製元素
    ground_line, = ax_main.plot([], [], '-', color=COLORS['ground'], lw=5,
                                 alpha=0.6, solid_capstyle='round')
    crank_line, = ax_main.plot([], [], '-', color=COLORS['crank'], lw=5,
                                solid_capstyle='round', label='曲柄 Crank', zorder=4)
    coupler_AB, = ax_main.plot([], [], '-', color=COLORS['coupler'], lw=4,
                                solid_capstyle='round', label='耦桿 Coupler', zorder=4)
    rocker_line, = ax_main.plot([], [], '-', color=COLORS['rocker'], lw=5,
                                 solid_capstyle='round', label='搖桿 Rocker', zorder=4)
    coupler_AP, = ax_main.plot([], [], '-', color=COLORS['coupler'], lw=3.5,
                                solid_capstyle='round', alpha=0.7, zorder=4)
    coupler_BP, = ax_main.plot([], [], '-', color=COLORS['coupler'], lw=3.5,
                                solid_capstyle='round', alpha=0.7, zorder=4)

    # 關節點（光暈效果）
    joint_A_glow, = ax_main.plot([], [], 'o', color=COLORS['crank'], markersize=18,
                                  alpha=0.2, zorder=5)
    joint_A, = ax_main.plot([], [], 'o', color=COLORS['crank'], markersize=8,
                             alpha=0.9, zorder=6)
    joint_B_glow, = ax_main.plot([], [], 'o', color=COLORS['rocker'], markersize=18,
                                  alpha=0.2, zorder=5)
    joint_B, = ax_main.plot([], [], 'o', color=COLORS['rocker'], markersize=8,
                             alpha=0.9, zorder=6)

    # 足端點（大光暈）
    foot_glow, = ax_main.plot([], [], 'o', color=COLORS['foot'], markersize=28,
                               alpha=0.15, zorder=7)
    foot_outer, = ax_main.plot([], [], 'o', color=COLORS['foot'], markersize=16,
                                alpha=0.8, zorder=8)
    foot_inner, = ax_main.plot([], [], 'o', color='white', markersize=6,
                                alpha=0.7, zorder=9, label='足端 P')

    # 固定支點
    draw_fixed_pivot(ax_main, linkage.O1, 'O₁')
    draw_fixed_pivot(ax_main, linkage.O2, 'O₂')

    # 地面
    ground_y = trajectory[:, 1].min() + 2
    ax_main.axhline(y=ground_y, color=COLORS['ground_line'], linestyle='-', alpha=0.4, lw=1.5)
    ax_main.fill_between([-65, 145], ground_y - 25, ground_y,
                          color=COLORS['ground_fill'], alpha=0.3)
    # 地面陰影線
    for gx in range(-60, 145, 12):
        ax_main.plot([gx, gx - 8], [ground_y, ground_y - 8],
                     '-', color=COLORS['ground_line'], alpha=0.15, lw=0.8)

    # 圖例
    legend = ax_main.legend(loc='upper right', fontsize=9, framealpha=0.4,
                            facecolor=COLORS['bg_panel'], edgecolor=COLORS['grid'],
                            labelcolor=COLORS['text'])

    # === 資訊面板 ===
    ax_info.set_facecolor(COLORS['bg_panel'])
    ax_info.set_xlim(0, 10)
    ax_info.set_ylim(0, 10)
    ax_info.axis('off')
    for spine in ax_info.spines.values():
        spine.set_visible(False)

    # 標題
    ax_info.text(5, 9.6, '[ Parameters ]', fontsize=14, fontweight='bold',
                 color=COLORS['text'], ha='center', va='center')

    # 分隔線
    ax_info.plot([0.5, 9.5], [9.2, 9.2], '-', color=COLORS['grid'], lw=1, alpha=0.6)

    # 連桿尺寸 —— 用色塊標示
    dim_items = [
        (8.6, '曲柄 Crank', f'{linkage.crank} mm', COLORS['crank']),
        (8.0, '耦桿 Coupler', f'{linkage.coupler} mm', COLORS['coupler']),
        (7.4, '機架 Ground', f'{linkage.ground} mm', COLORS['ground']),
        (6.8, '搖桿 Rocker', f'{linkage.rocker} mm', COLORS['rocker']),
    ]
    for y, label, val, color in dim_items:
        ax_info.plot(0.8, y, 's', color=color, markersize=10, alpha=0.8)
        ax_info.text(1.5, y, label, fontsize=10, color=COLORS['text_dim'],
                     va='center', family='monospace')
        ax_info.text(8.5, y, val, fontsize=11, color=color, fontweight='bold',
                     va='center', ha='right', family='monospace')

    # 分隔線
    ax_info.plot([0.5, 9.5], [6.3, 6.3], '-', color=COLORS['grid'], lw=1, alpha=0.6)

    # 步行特性
    stride_x = trajectory[:, 0].max() - trajectory[:, 0].min()
    stride_y = trajectory[:, 1].max() - trajectory[:, 1].min()

    ax_info.text(5, 5.9, '[ Gait Properties ]', fontsize=12, fontweight='bold',
                 color=COLORS['text'], ha='center')

    ax_info.text(1.2, 5.2, '步距 Stride', fontsize=10, color=COLORS['text_dim'],
                 va='center')
    ax_info.text(8.5, 5.2, f'{stride_x:.1f} mm', fontsize=13, fontweight='bold',
                 color=COLORS['highlight'], va='center', ha='right', family='monospace')

    ax_info.text(1.2, 4.5, '步高 Height', fontsize=10, color=COLORS['text_dim'],
                 va='center')
    ax_info.text(8.5, 4.5, f'{stride_y:.1f} mm', fontsize=13, fontweight='bold',
                 color=COLORS['highlight'], va='center', ha='right', family='monospace')

    # 分隔線
    ax_info.plot([0.5, 9.5], [3.9, 3.9], '-', color=COLORS['grid'], lw=1, alpha=0.6)

    # 動態文字
    angle_text = ax_info.text(5, 3.4, '', fontsize=11, family='monospace',
                               color=COLORS['text'], ha='center')
    foot_text = ax_info.text(5, 2.7, '', fontsize=10, family='monospace',
                              color=COLORS['text_dim'], ha='center')
    phase_text = ax_info.text(5, 1.8, '', fontsize=14, fontweight='bold',
                               ha='center', va='center')

    # === 足端軌跡小圖 ===
    ax_mini = fig.add_axes([0.60, 0.06, 0.33, 0.22])
    ax_mini.set_facecolor(COLORS['bg_axes'])
    for spine in ax_mini.spines.values():
        spine.set_color(COLORS['grid'])
        spine.set_linewidth(0.5)
    ax_mini.tick_params(colors=COLORS['text_dim'], labelsize=7)
    ax_mini.set_title('足端 P 軌跡', fontsize=9, color=COLORS['text'], pad=4)
    ax_mini.set_xlabel('X (mm)', fontsize=7, color=COLORS['text_dim'])
    ax_mini.set_ylabel('Y (mm)', fontsize=7, color=COLORS['text_dim'])
    ax_mini.set_aspect('equal')
    ax_mini.grid(True, alpha=0.1, color=COLORS['grid'])

    # 軌跡分段上色
    mid_y = (trajectory[:, 1].max() + trajectory[:, 1].min()) / 2
    walk_mask = trajectory[:, 1] < mid_y
    swing_mask = ~walk_mask

    ax_mini.plot(trajectory[:, 0], trajectory[:, 1], '-', color=COLORS['trace'],
                 lw=1.5, alpha=0.3)
    if np.any(walk_mask):
        walk_pts = trajectory[walk_mask]
        ax_mini.plot(walk_pts[:, 0], walk_pts[:, 1], '-', color=COLORS['support'],
                     lw=4, alpha=0.5, label='著地段')
    if np.any(swing_mask):
        swing_pts = trajectory[swing_mask]
        ax_mini.plot(swing_pts[:, 0], swing_pts[:, 1], '-', color=COLORS['swing'],
                     lw=3, alpha=0.4, label='擺動段')

    ax_mini.legend(fontsize=6, loc='upper left', framealpha=0.3,
                   facecolor=COLORS['bg_panel'], edgecolor=COLORS['grid'],
                   labelcolor=COLORS['text_dim'])

    mini_glow, = ax_mini.plot([], [], 'o', color=COLORS['foot'], markersize=12,
                               alpha=0.2, zorder=5)
    mini_dot, = ax_mini.plot([], [], 'o', color=COLORS['foot'], markersize=7,
                              zorder=6)

    trail_x, trail_y = [], []

    def update(frame):
        idx = frame % len(all_positions)
        theta, A, B, P = all_positions[idx]

        # 連桿
        ground_line.set_data([linkage.O1[0], linkage.O2[0]],
                              [linkage.O1[1], linkage.O2[1]])
        crank_line.set_data([linkage.O1[0], A[0]], [linkage.O1[1], A[1]])
        coupler_AB.set_data([A[0], B[0]], [A[1], B[1]])
        coupler_AP.set_data([A[0], P[0]], [A[1], P[1]])
        coupler_BP.set_data([B[0], P[0]], [B[1], P[1]])
        rocker_line.set_data([linkage.O2[0], B[0]], [linkage.O2[1], B[1]])

        # 關節光暈
        joint_A_glow.set_data([A[0]], [A[1]])
        joint_A.set_data([A[0]], [A[1]])
        joint_B_glow.set_data([B[0]], [B[1]])
        joint_B.set_data([B[0]], [B[1]])

        # 足端光暈
        foot_glow.set_data([P[0]], [P[1]])
        foot_outer.set_data([P[0]], [P[1]])
        foot_inner.set_data([P[0]], [P[1]])

        # 軌跡尾巴
        trail_x.append(P[0])
        trail_y.append(P[1])
        if len(trail_x) > 100:
            trail_x.pop(0)
            trail_y.pop(0)
        trace_line.set_data(trail_x, trail_y)

        # 小圖上的點
        mini_glow.set_data([P[0]], [P[1]])
        mini_dot.set_data([P[0]], [P[1]])

        # 動態文字
        deg = np.degrees(theta)
        angle_text.set_text(f'曲柄角度 θ = {deg:.0f}°')
        foot_text.set_text(f'P = ({P[0]:.1f}, {P[1]:.1f})')

        if P[1] < mid_y:
            phase_text.set_text('● 著地行走中')
            phase_text.set_color(COLORS['support'])
            foot_glow.set_color(COLORS['support'])
            foot_outer.set_color(COLORS['support'])
        else:
            phase_text.set_text('○ 擺動抬腳中')
            phase_text.set_color(COLORS['swing'])
            foot_glow.set_color(COLORS['swing'])
            foot_outer.set_color(COLORS['swing'])

        return []

    anim = FuncAnimation(fig, update, frames=360, interval=30, blit=False)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()
    return anim


# =====================================================
#  動畫 2：六足三角步態動畫
# =====================================================
def animate_hexapod():
    """六足三角步態動畫（6 個獨立側視圖 + 俯視）"""
    linkage = ChebyshevLinkage(crank=30, coupler=75, ground=75, rocker=75)
    trajectory, all_positions = linkage.get_trajectory(360)
    n = len(all_positions)

    # 六足配置：3行(前/中/後) × 2列(左/右)
    leg_configs = [
        {'name': 'L1-Front', 'side': 'L', 'x_offset': -60, 'y_row': 60,  'phase': 0,   'group': 'A', 'row': 0, 'col': 0},
        {'name': 'R4-Front', 'side': 'R', 'x_offset': 60,  'y_row': 60,  'phase': 180, 'group': 'B', 'row': 0, 'col': 1},
        {'name': 'L2-Mid',   'side': 'L', 'x_offset': -60, 'y_row': 0,   'phase': 180, 'group': 'B', 'row': 1, 'col': 0},
        {'name': 'R5-Mid',   'side': 'R', 'x_offset': 60,  'y_row': 0,   'phase': 0,   'group': 'A', 'row': 1, 'col': 1},
        {'name': 'L3-Rear',  'side': 'L', 'x_offset': -60, 'y_row': -60, 'phase': 0,   'group': 'A', 'row': 2, 'col': 0},
        {'name': 'R6-Rear',  'side': 'R', 'x_offset': 60,  'y_row': -60, 'phase': 180, 'group': 'B', 'row': 2, 'col': 1},
    ]

    mid_y = (trajectory[:, 1].max() + trajectory[:, 1].min()) / 2

    # === 佈局：左側 3×2 迷你側視圖 + 右側俯視圖 ===
    fig = plt.figure(figsize=(18, 10))
    fig.patch.set_facecolor(COLORS['bg_dark'])
    fig.suptitle('Chebyshev Hexapod -- Tripod Gait (6-Leg View)',
                 fontsize=17, fontweight='bold', color=COLORS['text'], y=0.98)

    # GridSpec: 3 行 × 3 列（左 2 列給迷你圖，右 1 列給俯視）
    gs = fig.add_gridspec(3, 3, width_ratios=[1, 1, 1.4],
                          hspace=0.30, wspace=0.25,
                          left=0.04, right=0.97, top=0.92, bottom=0.05)

    # 預計算軌跡範圍
    x_min, x_max = trajectory[:, 0].min(), trajectory[:, 0].max()
    y_min, y_max = trajectory[:, 1].min(), trajectory[:, 1].max()
    pad = 15
    ground_y = y_min + 2

    # ─── 建立 6 個迷你側視圖 ───
    leg_elements = {}

    for lc in leg_configs:
        ax = fig.add_subplot(gs[lc['row'], lc['col']])
        apply_dark_style(ax, '')

        # 標題帶組別標記
        grp_color = COLORS['support'] if lc['group'] == 'A' else COLORS['swing']
        ax.set_title(f"{lc['name']}  [{lc['group']}]", fontsize=10,
                     fontweight='bold', color=grp_color, pad=4)

        ax.set_xlim(-40, 130)
        ax.set_ylim(y_min - pad, max(y_max, 0) + pad)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.08, color=COLORS['grid'], linewidth=0.4)
        ax.tick_params(labelsize=7)

        # 地面
        ax.axhline(y=ground_y, color=COLORS['ground_line'], linestyle='-', alpha=0.3, lw=1)
        ax.fill_between([-40, 130], ground_y - 10, ground_y,
                         color=COLORS['ground_fill'], alpha=0.25)

        # 軌跡背景
        ax.plot(trajectory[:, 0], trajectory[:, 1], '-', color=COLORS['trace'],
                alpha=0.12, lw=1.5)

        # 固定支點（簡化版）
        for pos in [linkage.O1, linkage.O2]:
            ax.plot(pos[0], pos[1], 'o', color=COLORS['ground'], markersize=6, alpha=0.5)

        # 連桿元素
        elems = {}
        elems['crank'], = ax.plot([], [], '-', color=COLORS['crank'], lw=3.5,
                                   solid_capstyle='round', zorder=4)
        elems['coupler_AB'], = ax.plot([], [], '-', color=COLORS['coupler'], lw=3,
                                        solid_capstyle='round', zorder=4)
        elems['coupler_AP'], = ax.plot([], [], '-', color=COLORS['coupler'], lw=2,
                                        alpha=0.6, solid_capstyle='round', zorder=3)
        elems['coupler_BP'], = ax.plot([], [], '-', color=COLORS['coupler'], lw=2,
                                        alpha=0.6, solid_capstyle='round', zorder=3)
        elems['rocker'], = ax.plot([], [], '-', color=COLORS['rocker'], lw=3.5,
                                    solid_capstyle='round', zorder=4)

        # 關節
        elems['joint_A'], = ax.plot([], [], 'o', color=COLORS['crank'], markersize=5, zorder=6)
        elems['joint_B'], = ax.plot([], [], 'o', color=COLORS['rocker'], markersize=5, zorder=6)

        # 足端（光暈 + 實心）
        elems['foot_glow'], = ax.plot([], [], 'o', markersize=18, alpha=0.2, zorder=7)
        elems['foot'], = ax.plot([], [], 'o', markersize=10, alpha=0.9, zorder=8)

        # 狀態文字（左下角）
        elems['status'] = ax.text(0.04, 0.06, '', transform=ax.transAxes,
                                   fontsize=9, fontweight='bold', va='bottom')

        # 角度文字（右下角）
        elems['angle'] = ax.text(0.96, 0.06, '', transform=ax.transAxes,
                                  fontsize=8, color=COLORS['text_dim'],
                                  va='bottom', ha='right', family='monospace')

        elems['ax'] = ax
        elems['phase'] = lc['phase']
        leg_elements[lc['name']] = elems

    # ─── 俯視圖（右側，佔 3 行高） ───
    ax_top = fig.add_subplot(gs[:, 2])
    apply_dark_style(ax_top, 'Top View -- Tripod Gait')
    ax_top.set_xlim(-120, 120)
    ax_top.set_ylim(-110, 110)
    ax_top.set_aspect('equal')
    ax_top.grid(True, alpha=0.08, color=COLORS['grid'], linewidth=0.5)
    ax_top.set_xlabel('L / R (mm)', fontsize=10)
    ax_top.set_ylabel('F / B (mm)', fontsize=10)

    # 六邊形機身
    hex_verts = []
    for i in range(6):
        angle = np.pi / 6 + i * np.pi / 3
        hex_verts.append([35 * np.cos(angle), 50 * np.sin(angle)])
    hex_verts.append(hex_verts[0])
    hex_x = [v[0] for v in hex_verts]
    hex_y = [v[1] for v in hex_verts]
    ax_top.fill(hex_x, hex_y, color=COLORS['body'], alpha=0.2)
    ax_top.plot(hex_x, hex_y, '-', color=COLORS['body'], lw=2, alpha=0.5)
    ax_top.text(0, 0, 'BODY', ha='center', va='center',
                fontsize=11, fontweight='bold', color=COLORS['text_dim'], alpha=0.5)

    # 前進方向
    ax_top.annotate('', xy=(0, 100), xytext=(0, 80),
                    arrowprops=dict(arrowstyle='->', color=COLORS['accent'],
                                    lw=2.5, mutation_scale=20))
    ax_top.text(0, 106, 'FWD', ha='center', fontsize=10,
                color=COLORS['accent'], fontweight='bold')

    # 六足俯視元素
    top_feet = {}
    top_feet_glow = {}
    top_legs = {}
    for lc in leg_configs:
        glow, = ax_top.plot([], [], 'o', markersize=24, alpha=0.15, zorder=4)
        top_feet_glow[lc['name']] = glow
        dot, = ax_top.plot([], [], 'o', markersize=16, zorder=5)
        top_feet[lc['name']] = dot
        line, = ax_top.plot([], [], '-', color=COLORS['body'], lw=2.5, alpha=0.4)
        top_legs[lc['name']] = line
        # 標籤
        short_name = lc['name'].split('-')[0]  # L1, R4, etc.
        ax_top.text(lc['x_offset'], lc['y_row'], short_name,
                    ha='center', va='center', fontsize=7,
                    fontweight='bold', color='white', zorder=7, alpha=0.9)

    # 三角形支撐區
    tri_fill_A = ax_top.fill([], [], color=COLORS['support'], alpha=0.08)[0]
    tri_fill_B = ax_top.fill([], [], color=COLORS['swing'], alpha=0.06)[0]
    tri_line_A, = ax_top.plot([], [], '-', color=COLORS['support'], lw=2.5, alpha=0.5)
    tri_line_B, = ax_top.plot([], [], '--', color=COLORS['swing'], lw=2, alpha=0.4)

    phase_label = ax_top.text(0, -100, '', ha='center', fontsize=10,
                               fontweight='bold', color=COLORS['text'])

    def get_foot_state(frame_idx, phase_offset):
        idx = (frame_idx + int(phase_offset / 360 * n)) % n
        if idx < len(all_positions):
            _, A, B, P = all_positions[idx]
            is_ground = P[1] < mid_y
            return A, B, P, is_ground
        return None, None, None, False

    def update(frame):
        support_pts = []
        swing_pts = []

        # === 更新每隻腳的迷你側視圖 ===
        for lc in leg_configs:
            A, B, P, is_ground = get_foot_state(frame, lc['phase'])
            elems = leg_elements[lc['name']]
            if A is None:
                continue

            # 連桿
            elems['crank'].set_data([linkage.O1[0], A[0]], [linkage.O1[1], A[1]])
            elems['coupler_AB'].set_data([A[0], B[0]], [A[1], B[1]])
            elems['coupler_AP'].set_data([A[0], P[0]], [A[1], P[1]])
            elems['coupler_BP'].set_data([B[0], P[0]], [B[1], P[1]])
            elems['rocker'].set_data([linkage.O2[0], B[0]], [linkage.O2[1], B[1]])

            # 關節
            elems['joint_A'].set_data([A[0]], [A[1]])
            elems['joint_B'].set_data([B[0]], [B[1]])

            # 足端
            color = COLORS['support'] if is_ground else COLORS['swing']
            elems['foot_glow'].set_data([P[0]], [P[1]])
            elems['foot_glow'].set_color(color)
            elems['foot'].set_data([P[0]], [P[1]])
            elems['foot'].set_color(color)

            # 狀態文字
            if is_ground:
                elems['status'].set_text('GROUND')
                elems['status'].set_color(COLORS['support'])
            else:
                elems['status'].set_text('SWING')
                elems['status'].set_color(COLORS['swing'])

            # 角度
            theta_idx = (frame + int(lc['phase'] / 360 * n)) % n
            if theta_idx < len(all_positions):
                theta = all_positions[theta_idx][0]
                elems['angle'].set_text(f'{np.degrees(theta):.0f} deg')

            # 邊框顏色隨狀態變化
            border_color = COLORS['support'] if is_ground else COLORS['swing']
            for spine in elems['ax'].spines.values():
                spine.set_color(border_color)
                spine.set_linewidth(1.5)

            # 俯視圖數據
            dot = top_feet[lc['name']]
            glow = top_feet_glow[lc['name']]
            line = top_legs[lc['name']]

            dot.set_data([lc['x_offset']], [lc['y_row']])
            glow.set_data([lc['x_offset']], [lc['y_row']])
            line.set_data([0, lc['x_offset']], [lc['y_row'], lc['y_row']])

            if is_ground:
                dot.set_color(COLORS['support'])
                glow.set_color(COLORS['support'])
                dot.set_markersize(18)
                glow.set_markersize(30)
                glow.set_alpha(0.2)
                support_pts.append((lc['x_offset'], lc['y_row']))
            else:
                dot.set_color(COLORS['swing'])
                glow.set_color(COLORS['swing'])
                dot.set_markersize(13)
                glow.set_markersize(22)
                glow.set_alpha(0.1)
                swing_pts.append((lc['x_offset'], lc['y_row']))

        # === 俯視圖三角支撐 ===
        if len(support_pts) >= 3:
            pts = support_pts + [support_pts[0]]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            tri_line_A.set_data(xs, ys)
            tri_fill_A.set_xy(list(zip(xs, ys)))
        else:
            tri_line_A.set_data([], [])
            tri_fill_A.set_xy([(0, 0)])

        if len(swing_pts) >= 3:
            pts = swing_pts + [swing_pts[0]]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            tri_line_B.set_data(xs, ys)
            tri_fill_B.set_xy(list(zip(xs, ys)))
        else:
            tri_line_B.set_data([], [])
            tri_fill_B.set_xy([(0, 0)])

        # 相位標籤
        _, _, _, is_g = get_foot_state(frame, 0)
        if is_g:
            phase_label.set_text('[A] GND  |  [B] SWG')
            phase_label.set_color(COLORS['support'])
        else:
            phase_label.set_text('[B] GND  |  [A] SWG')
            phase_label.set_color(COLORS['swing'])

        return []

    anim = FuncAnimation(fig, update, frames=n, interval=30, blit=False)
    plt.show()
    return anim


# =====================================================
#  靜態圖 3：參數比較
# =====================================================
def compare_parameters():
    """不同連桿參數的足端軌跡比較"""
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    fig.patch.set_facecolor(COLORS['bg_dark'])
    fig.suptitle('Chebyshev Linkage — 參數比較',
                 fontsize=17, fontweight='bold', color=COLORS['text'], y=0.97)

    configs = [
        {'crank': 25, 'coupler': 62.5, 'ground': 62.5, 'rocker': 62.5,
         'title': '小尺寸 (1:2.5 經典比)', 'highlight': False},
        {'crank': 30, 'coupler': 75, 'ground': 75, 'rocker': 75,
         'title': '★ 建議尺寸 (30/75/75/75)', 'highlight': True},
        {'crank': 35, 'coupler': 87.5, 'ground': 87.5, 'rocker': 87.5,
         'title': '大尺寸 (1:2.5 經典比)', 'highlight': False},
        {'crank': 30, 'coupler': 90, 'ground': 75, 'rocker': 75,
         'title': '長耦桿 (30/90/75/75)', 'highlight': False},
    ]

    for ax, cfg in zip(axes.flat, configs):
        apply_dark_style(ax)
        ax.grid(True, alpha=0.1, color=COLORS['grid'], linewidth=0.5)

        linkage = ChebyshevLinkage(cfg['crank'], cfg['coupler'],
                                    cfg['ground'], cfg['rocker'])
        traj, _ = linkage.get_trajectory(500)

        if len(traj) == 0:
            ax.text(0.5, 0.5, '無法運動\n（死點）', transform=ax.transAxes,
                    ha='center', va='center', fontsize=14, color=COLORS['swing'])
            ax.set_title(cfg['title'], color=COLORS['text_dim'])
            continue

        mid_y = (traj[:, 1].max() + traj[:, 1].min()) / 2
        walk_mask = traj[:, 1] < mid_y

        # 完整軌跡
        ax.plot(traj[:, 0], traj[:, 1], '-', color=COLORS['trace'], lw=2, alpha=0.5)

        if np.any(walk_mask):
            walk_pts = traj[walk_mask]
            ax.plot(walk_pts[:, 0], walk_pts[:, 1], '-', color=COLORS['support'],
                    lw=5, alpha=0.6, label='著地段')
        if np.any(~walk_mask):
            swing_pts = traj[~walk_mask]
            ax.plot(swing_pts[:, 0], swing_pts[:, 1], '-', color=COLORS['swing'],
                    lw=3.5, alpha=0.45, label='擺動段')

        # 固定支點
        ax.plot(*linkage.O1, 's', color=COLORS['ground'], markersize=8, alpha=0.7)
        ax.plot(*linkage.O2, 's', color=COLORS['ground'], markersize=8, alpha=0.7)
        ax.plot([linkage.O1[0], linkage.O2[0]], [linkage.O1[1], linkage.O2[1]],
                '-', color=COLORS['ground'], lw=2.5, alpha=0.5)

        stride_x = traj[:, 0].max() - traj[:, 0].min()
        stride_y = traj[:, 1].max() - traj[:, 1].min()

        # 標題高亮
        title_color = COLORS['highlight'] if cfg['highlight'] else COLORS['text']
        ax.set_title(cfg['title'], fontweight='bold', color=title_color, fontsize=11, pad=8)

        # 高亮邊框
        if cfg['highlight']:
            for spine in ax.spines.values():
                spine.set_color(COLORS['highlight'])
                spine.set_linewidth(2)

        ax.set_aspect('equal')
        ax.set_xlabel('X (mm)', fontsize=9)
        ax.set_ylabel('Y (mm)', fontsize=9)

        info = f'步距 ≈ {stride_x:.0f} mm   步高 ≈ {stride_y:.0f} mm'
        ax.text(0.03, 0.04, info, transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom', color=COLORS['text'],
                bbox=dict(boxstyle='round,pad=0.4', facecolor=COLORS['bg_panel'],
                          edgecolor=COLORS['grid'], alpha=0.9))

        ax.legend(fontsize=7, loc='upper right', framealpha=0.4,
                  facecolor=COLORS['bg_panel'], edgecolor=COLORS['grid'],
                  labelcolor=COLORS['text'])

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.show()


# =====================================================
#  主選單
# =====================================================
def main():
    banner = """
╔══════════════════════════════════════════════════════╗
║  Chebyshev Lambda Linkage 六足步行機構 — 可視化模型  ║
║  2026 NCKU 機械專題實作                              ║
╚══════════════════════════════════════════════════════╝
"""
    print(banner)
    print("  [1] 單足連桿動畫（含軌跡 + 參數資訊）")
    print("  [2] 六足三角步態動畫（側視 + 俯視）")
    print("  [3] 參數比較圖（不同連桿尺寸）")
    print("  [q] 離開")
    print()

    while True:
        choice = input("  請選擇功能 (1/2/3/q): ").strip()

        if choice == '1':
            print("  ▸ 啟動單足連桿動畫...")
            animate_single_leg()
        elif choice == '2':
            print("  ▸ 啟動六足步態動畫...")
            animate_hexapod()
        elif choice == '3':
            print("  ▸ 產生參數比較圖...")
            compare_parameters()
        elif choice.lower() == 'q':
            print("  再見！")
            break
        else:
            print("  ✗ 無效選擇，請輸入 1, 2, 3 或 q")


if __name__ == '__main__':
    main()
