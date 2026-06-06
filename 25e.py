import math
import time as _std_time
import struct

from maix import camera, display, image, app, time

# =============================================================================
# 日志
# =============================================================================
_boot = _std_time.time()
LOG_LEVEL = 1
_LT = ["TRACE","DEBUG"," INFO","ERROR"]
def _log(lv, mod, msg):
    if lv >= LOG_LEVEL:
        ms = int((_std_time.time() - _boot) * 1000)
        print(f"[{ms:>8}ms][{_LT[lv]}][{mod:>5}] {msg}")
def log_i(m,s): _log(2,m,s)
def log_e(m,s): _log(3,m,s)

# =============================================================================
# 参数
# =============================================================================
CAM_W, CAM_H = 320, 240
LOCK_MAX_JUMP = 80
ROI_SIZE = 30 # 缩小放大镜尺寸，降低边缘与反光干扰
CORNER_DEADZONE_SQ = 16  # 角点死区半径²（4px），小于此距离时复用上帧角点
SMOOTH_ALPHA = 0.25       # 中心点EMA平滑系数 (0=极平滑/高延迟, 1=无平滑/原始值)

COLOR_TH = {
    "tape_bin":   {"gray": [[0, 115]],   "note": "黑色胶布灰度阈值 (全图二值化用)"},
    "blob_white": {"gray": [[200, 255]], "note": "二值图白色区域 (找白框)"},
    "blob_black": {"gray": [[0, 50]],    "note": "二值图黑色区域 (找黑芯)"},
}

DEBUG_FAIL_REASON = "Init"

# =============================================================================
# 工具函数：计算对角线交点 (抵抗透视畸变)
# =============================================================================
def get_diagonal_intersection(tl, tr, br, bl):
    """计算两条对角线(tl->br 和 tr->bl)的交点，得出真实的透视物理中心"""
    x1, y1 = tl
    x2, y2 = br
    x3, y3 = tr
    x4, y4 = bl
    
    # 直线1 (tl-br): A1*x + B1*y = C1
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1
    
    # 直线2 (tr-bl): A2*x + B2*y = C2
    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3
    
    det = A1 * B2 - A2 * B1
    if det == 0:
        return (x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4
    else:
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        return int(x), int(y)

# =============================================================================
# 动态透视角点检测：旋转不变曼哈顿法 (动态基底投影)
# =============================================================================
def get_robust_corners(blob, safe_bottom_y=None):
    """利用动态向量点乘，在任意旋转角度下精准提取物理尖角"""
    try:
        pts = blob.corners() 
    except AttributeError:
        pts = blob.mini_corners()
    
    if not pts or len(pts) < 4:
        pts = blob.mini_corners()
        
    if not pts or len(pts) < 4:
        return None

    if safe_bottom_y is not None:
        filt = [p for p in pts if p[1] < safe_bottom_y]
        if len(filt) >= 4:
            pts = filt

    theta = blob.rotation()

    angles = [
        theta + math.pi/4,       
        theta + 3*math.pi/4,     
        theta + 5*math.pi/4,     
        theta + 7*math.pi/4      
    ]
    
    corners = []
    for a in angles:
        vx, vy = math.cos(a), math.sin(a)
        best_pt = max(pts, key=lambda p: p[0]*vx + p[1]*vy)
        corners.append((int(best_pt[0]), int(best_pt[1])))

    corners = list(set(corners))
    if len(corners) < 4:
        corners = [(int(p[0]), int(p[1])) for p in blob.mini_corners()]
        
    if not corners or len(corners) < 4:
        return None

    tl = min(corners, key=lambda p: p[0] + p[1])  
    br = max(corners, key=lambda p: p[0] + p[1])  
    tr = max(corners, key=lambda p: p[0] - p[1])  
    bl = min(corners, key=lambda p: p[0] - p[1])  
    
    return [tl, tr, br, bl]

# =============================================================================
# 核心识别: 漏斗式筛选 + 局部极径最大化
# =============================================================================
def find_target(bin_img, hint_roi=None, last_corners=None):
    global DEBUG_FAIL_REASON
    DEBUG_FAIL_REASON = "未找到任何白框"
    search = {}
    if hint_roi:
        search['roi'] = hint_roi
        
    white_blobs = bin_img.find_blobs(COLOR_TH["blob_white"]["gray"], 
                                      area_threshold=100, pixels_threshold=20,
                                      merge=True, x_stride=1, y_stride=1, **search)
    if not white_blobs:
        return None
    
    valid_tapes = []
    for b in white_blobs:
        area = b.w() * b.h()
        if area > 76000 or area < 400:
            DEBUG_FAIL_REASON = f"白框面积越界({area})"
            continue
        ratio = b.w() / b.h()
        if ratio < 0.4 or ratio > 5.0:
            DEBUG_FAIL_REASON = f"白框比例越界({ratio:.1f})"
            continue
            
        valid_tapes.append(b)
        
    if not valid_tapes:
        return None
    
    valid_tapes.sort(key=lambda b: b.w() * b.h(), reverse=True)
    
    for tape in valid_tapes:
        roi = tape.rect()
        
        black_blobs = bin_img.find_blobs(COLOR_TH["blob_black"]["gray"], 
                                          roi=roi, area_threshold=5,
                                          pixels_threshold=5, merge=False, 
                                          x_stride=2, y_stride=2)
        if not black_blobs:
            DEBUG_FAIL_REASON = "框内无黑芯"
            continue
            
        candidates = []
        for bb in black_blobs:
            # 【抗纯白背景干扰：严格黑芯自身过滤】
            # 靶子核心一定是接近正方形的。如果背景全白，导致整个视野被判定为白框，
            # 这里的黑芯比例限制可以瞬间屏蔽掉“宽广的地板”或“细长的支架”等环境阴影。
            core_ratio = bb.w() / (bb.h() + 1e-5)
            if core_ratio < 0.3 or core_ratio > 3.0:
                DEBUG_FAIL_REASON = "黑芯几何比例畸形(疑为阴影/地板)"
                continue
                
            if bb.w() * bb.h() > 45000:
                DEBUG_FAIL_REASON = "黑芯面积过大(疑为环境阴影)"
                continue

            # 允许边缘溢出（兼容反光导致的胶带断裂），但限制重心必须在中央区域，拒绝四角背景
            if not (tape.x() + tape.w() * 0.2 < bb.cx() < tape.x() + tape.w() * 0.8):
                DEBUG_FAIL_REASON = "黑芯重心偏离X(疑为背景)"
                continue
            if bb.cy() < tape.y() + tape.h() * 0.15:
                DEBUG_FAIL_REASON = "黑芯重心偏离Y(疑为背景)"
                continue
            if (bb.w() * bb.h()) < (tape.w() * tape.h()) * 0.15: 
                DEBUG_FAIL_REASON = "黑芯相对面积太小(<15%)"
                continue
            
            candidates.append(bb)
            
        if candidates:
            valid_cores = [bb for bb in candidates if bb.cy() < tape.y() + tape.h() * 0.75]
            if not valid_cores: 
                DEBUG_FAIL_REASON = "黑芯位置全偏下(疑为干扰)"
                continue
            if sum(b.pixels() for b in valid_cores) == 0: continue
            
            # 几何硬截断：通过内芯高度推算真实外框的底边极限位置，切断三脚架粘连
            core_min_y = min(b.y() for b in valid_cores)
            core_max_y = max(b.y() + b.h() for b in valid_cores)
            safe_bottom_y = core_max_y + max(0, core_min_y - tape.y()) * 2.5

            # ================================================================
            # 局部放大镜钉死角点 (升级版：引入几何与物理防畸变约束)
            # ================================================================
            corners = None
            if last_corners and not any(p[1] > safe_bottom_y for p in last_corners):
                corners = []
                keys = [lambda p: p[0]+p[1], lambda p: p[0]-p[1], lambda p: p[0]+p[1], lambda p: p[0]-p[1]]
                is_max = [False, True, True, False]
                
                for i in range(4):
                    lcx, lcy = last_corners[i]
                    rx = max(0, lcx - ROI_SIZE//2)
                    ry = max(0, lcy - ROI_SIZE//2)
                    rw = min(CAM_W - rx, ROI_SIZE)
                    rh = min(CAM_H - ry, ROI_SIZE)
                    
                    if rw <= 0 or rh <= 0:
                        break
                        
                    roi_local = (rx, ry, rw, rh)
                    
                    # 【约束1：局部面积过滤】提升阈值，防止抓取极小噪点
                    blobs = bin_img.find_blobs(COLOR_TH["blob_white"]["gray"], roi=roi_local, 
                                               area_threshold=15, pixels_threshold=15, merge=True)
                    if not blobs: break
                    
                    b = max(blobs, key=lambda x: x.pixels())
                    pts = b.mini_corners() or b.corners()
                    if not pts: break
                    
                    best = max(pts, key=keys[i]) if is_max[i] else min(pts, key=keys[i])
                    corners.append((int(best[0]), int(best[1])))
                
                # 【约束2：四边形几何拓扑审查】
                if corners is not None and len(corners) == 4:
                    tl, tr, br, bl = corners
                    
                    # 计算四条边的长度平方 (省去开平方根的性能消耗)
                    dist_top_sq = (tl[0] - tr[0])**2 + (tl[1] - tr[1])**2
                    dist_bot_sq = (bl[0] - br[0])**2 + (bl[1] - br[1])**2
                    dist_lft_sq = (tl[0] - bl[0])**2 + (tl[1] - bl[1])**2
                    dist_rgt_sq = (tr[0] - br[0])**2 + (tr[1] - br[1])**2
                    
                    # 审查A：防整体萎缩 (边长必须大于阈值)
                    MIN_EDGE_SQ = 30 ** 2 
                    if min(dist_top_sq, dist_bot_sq, dist_lft_sq, dist_rgt_sq) < MIN_EDGE_SQ:
                        corners = None 
                    else:
                        # 审查B：防单边被严重拉长 (对边比例必须在合理透视范围内)
                        ratio_tb = dist_top_sq / (dist_bot_sq + 1e-5)
                        ratio_lr = dist_lft_sq / (dist_rgt_sq + 1e-5)
                        
                        if not (0.2 < ratio_tb < 5.0) or not (0.2 < ratio_lr < 5.0):
                            DEBUG_FAIL_REASON = "局部角点四边形比例异常"
                            corners = None 
                else:
                    DEBUG_FAIL_REASON = "局部未集齐4个角点或太小"
                    corners = None
            # ================================================================

            if not corners:
                corners = get_robust_corners(tape, safe_bottom_y)

            # 退化检测：4个角点必须两两不同，否则对角线交点无意义
            if corners and len(set(corners)) < 4:
                DEBUG_FAIL_REASON = "角点退化(含重复坐标)"
                corners = get_robust_corners(tape, safe_bottom_y)
                if corners and len(set(corners)) < 4:
                    corners = None

            if not corners:
                continue

            # 角点死区：单帧位移 < CORNER_DEADZONE_SQ 时复用上帧角点，切断局部放大镜反馈震荡
            if last_corners and len(last_corners) == 4:
                corners = [
                    old if (new[0]-old[0])**2 + (new[1]-old[1])**2 < CORNER_DEADZONE_SQ else new
                    for new, old in zip(corners, last_corners)
                ]

            # 中心点：四角点对角线交点（抗透视畸变），跳变保护交由 LOCK_MAX_JUMP 处理
            tl, tr, br, bl = corners
            cx, cy = get_diagonal_intersection(tl, tr, br, bl)
            inner_area = sum(b.w() * b.h() for b in valid_cores)
            
            return {
                "corners": corners,
                "outer_rect": tape.rect(),
                "inner_area": inner_area,
                "cx": cx, "cy": cy
            }
            
    return None

# =============================================================================
# 主程序
# =============================================================================
try:
    cam = camera.Camera(CAM_W, CAM_H, image.Format.FMT_GRAYSCALE)
except Exception:
    cam = camera.Camera(CAM_W, CAM_H) 

disp = display.Display()
frame_count = 0
last_roi = None
last_corners = None  

# =============================================================================
# 串口初始化
# =============================================================================
uart_ports = []
try:
    from maix import uart
    devs = uart.list_devices()
    log_i("MAIN", f"底层端口大全: {devs}")
    
    try:
        from maix import pinmap
        try:
            pinmap.set_pin_function("B0", "UART2_TX")
            pinmap.set_pin_function("B1", "UART2_RX")
        except Exception as e:
            log_i("MAIN", f"Pinmap 提示: {e}")
            
        port = uart.UART("/dev/ttyS2", 115200)
        uart_ports.append(port)
        log_i("MAIN", "已挂载 MaixCAM2 U2 (/dev/ttyS2) 串口")
    except Exception as e:
        log_e("MAIN", f"/dev/ttyS2 打开失败: {e}")
except Exception as e:
    log_e("MAIN", f"串口外接失败: {e}")

sm_cx, sm_cy, sm_area = None, None, None
screen_cx, screen_cy = CAM_W // 2, CAM_H // 2

is_tracking = False
_t0_ms = time.ticks_ms()
roi_miss_count = 0

# UART RX 计数 (每收到 1 字节自增，配合 STM32 按键发送 'T' 验证 USART6 通信)
uart_rx_count = 0

def _ts():
    return int(time.ticks_ms() - _t0_ms)

print(f"[T:{_ts():06d}] SYS_START screen_ctr=({screen_cx},{screen_cy})")

try:
    while not app.need_exit():
        try:
            img = cam.read()
        except Exception as e:
            print(f"[T:{_ts():06d}] CAM_ERR 读取异常: {e}")
            _std_time.sleep(0.05) 
            continue
            
        if img is None:
            continue
        frame_count += 1

        if img.format() != image.Format.FMT_GRAYSCALE:
            try:
                img = img.to_format(image.Format.FMT_GRAYSCALE)
            except AttributeError:
                img = img.to_grayscale()

        # 直接拷贝原始灰度图用于屏幕显示，不转 RGB 以极致节省帧率
        display_img = img.copy()

        img.binary(COLOR_TH["tape_bin"]["gray"])
        # 已去掉形态学处理，进一步节省帧率
        bin_img = img

        tgt = find_target(bin_img, last_roi, last_corners)
        
        if tgt and tgt["inner_area"] < 300:
            tgt = None

        if tgt is None and last_roi is not None:
            rx, ry, rw, rh = last_roi
            cx, cy = rx + rw // 2, ry + rh // 2
            
            new_w = int(rw * 1.3)
            new_h = int(rh * 1.3)
            
            if new_w >= CAM_W or new_h >= CAM_H:
                last_roi = None
                last_corners = None 
                tgt = find_target(bin_img)
                if tgt and tgt["inner_area"] < 300:
                    tgt = None
            else:
                new_rx = max(0, cx - new_w // 2)
                new_ry = max(0, cy - new_h // 2)
                new_w = min(CAM_W - new_rx, new_w)
                new_h = min(CAM_H - new_ry, new_h)
                
                last_roi = (new_rx, new_ry, new_w, new_h)
                tgt = find_target(bin_img, last_roi, last_corners)
                if tgt and tgt["inner_area"] < 300:
                    tgt = None

        if uart_ports:
            for p in uart_ports:
                try:
                    in_data = p.read()
                    if in_data:
                        # 每接收 1 字节就累加 RX 计数并打印 (供 STM32 按键测试用)
                        uart_rx_count += len(in_data)
                        print(f"[T:{_ts():06d}] UART_RX +{len(in_data)} RX={uart_rx_count} raw={bytes(in_data)}")
                        if b'T' in in_data:
                            is_tracking = True
                            print(f"[T:{_ts():06d}] UART_RX CMD=T TRACKING_ON")
                        if b'U' in in_data:
                            is_tracking = False
                            print(f"[T:{_ts():06d}] UART_RX CMD=U TRACKING_OFF")
                except Exception as e:
                    if frame_count % 100 == 0:
                        print(f"[T:{_ts():06d}] UART_ERR {e}")
                        
        if tgt:
            raw_cx, raw_cy = tgt["cx"], tgt["cy"]
            if sm_cx is not None and sm_cy is not None:
                jump = math.sqrt((raw_cx - sm_cx)**2 + (raw_cy - sm_cy)**2)
                if jump > LOCK_MAX_JUMP:
                    DEBUG_FAIL_REASON = f"中心跳动过大({jump:.1f})"
                    tgt = None
                    
        if tgt:
            if sm_cx is None:
                sm_cx, sm_cy = float(raw_cx), float(raw_cy)
            else:
                sm_cx = sm_cx * (1 - SMOOTH_ALPHA) + raw_cx * SMOOTH_ALPHA
                sm_cy = sm_cy * (1 - SMOOTH_ALPHA) + raw_cy * SMOOTH_ALPHA
            sm_area = tgt["inner_area"]

            cx, cy = int(sm_cx), int(sm_cy)
            corners = tgt["corners"]
            last_corners = corners 
            
            ox, oy, ow, oh = tgt["outer_rect"]
            ex = max(60, ow // 2)
            ey = max(60, oh // 2)
            rx = max(0, ox - ex)
            ry = max(0, oy - ey)
            last_roi = (rx, ry,
                        min(ow + 2*ex, CAM_W - rx),
                        min(oh + 2*ey, CAM_H - ry))
            
            for c in corners:
                display_img.draw_circle(c[0], c[1], 5, color=image.COLOR_WHITE, thickness=2)
                display_img.draw_rect(max(0, c[0]-ROI_SIZE//2), max(0, c[1]-ROI_SIZE//2), ROI_SIZE, ROI_SIZE, color=image.COLOR_WHITE, thickness=1)
                
            # 将四个角点首尾相连，绘制闭合的包围矩形
            for i in range(4):
                p1 = corners[i]
                p2 = corners[(i + 1) % 4]
                display_img.draw_line(p1[0], p1[1], p2[0], p2[1], color=image.COLOR_WHITE, thickness=2)
                
            display_img.draw_cross(cx, cy, color=image.COLOR_WHITE, size=15, thickness=1)
            display_img.draw_cross(screen_cx, screen_cy, color=image.COLOR_BLACK, size=10, thickness=1)

            dx = cx - screen_cx
            dy = cy - screen_cy

            if uart_ports:
                try:
                    payload = struct.pack("<BBhhB", 0x55, 0xAA, int(dx), int(dy), 0x0A)
                    for p in uart_ports:
                        p.write(payload)
                except Exception as e:
                    if frame_count % 50 == 0:
                        log_e("MAIN", f"打包或发送异常: {e}")
        
        fps = time.fps()
        state = "LOCK" if tgt else "LOST"

        if tgt:
            tgt_area = tgt["inner_area"] 
            roi_miss_count = 0
            if frame_count % 3 == 0:
                print(f"[T:{_ts():06d}] VIS {state} fps={fps:.0f} cx={cx} cy={cy} dx={dx:+d} dy={dy:+d} area={tgt_area} corners={tgt['corners']}")
        else:
            roi_miss_count += 1
            if roi_miss_count > 15:
                sm_cx, sm_cy = None, None
                last_roi = None
                last_corners = None 
                
            if frame_count % 15 == 0: 
                print(f"[T:{_ts():06d}] VIS {state} fps={fps:.0f} reason={DEBUG_FAIL_REASON}")
            
        # 恢复输出带有标记的原始灰度图 (img 已经被二值化破坏，必须用 display_img)
        disp.show(display_img)
        
        if frame_count % 150 == 0: 
            print(f"[T:{_ts():06d}] HEARTBEAT fps={fps:.0f} ctr=({screen_cx},{screen_cy})")

except Exception as e:
    log_e("MAIN", f"异常: {e}")
    import traceback
    traceback.print_exc()
finally:
    log_i("MAIN", "系统关闭")