# 1280x960 camera input version
# 注意：YOLO kmodel 仍然是 320x320 输入，AI2D 会把 1280x960 resize 到 320x320
# LCD/UI 仍然是 640x480
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d

from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import gc
import time
import utime
import aidemo
from machine import I2C, FPIOA

from ybUtils.YbKey import YbKey


# ==========================================================
# INA226 电流 / 功率监测
# ==========================================================
INA226_ADDR = 0x40

REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

RSHUNT = 0.0436
SHUNT_LSB_V = 2.5e-6
CURRENT_LSB = 0.0001
CAL_VALUE = int(0.00512 / (CURRENT_LSB * RSHUNT))


class INA226Monitor:
    def __init__(self):
        self.i2c = None
        self.devices = []
        self.ina_found = False
        self.init_ok = False
        self.ready = False
        self.last_scan_ms = 0
        self.error_text = ""

        self.smooth_current_shunt = None
        self.smooth_current_reg = None
        self.alpha = 0.25

        self.current_mA = 0.0
        self.current_reg_mA = 0.0
        self.power_W = 0.0
        self.shunt_mV = 0.0
        self.raw_shunt = 0
        self.bus_V = 0.0
        self.raw_bus = 0

        self.setup()

    def setup(self):
        try:
            fpioa = FPIOA()

            fpioa.set_function(
                34,
                FPIOA.IIC1_SCL,
                oe=1,
                ie=1,
                pu=1,
                st=1,
                ds=15,
            )

            fpioa.set_function(
                35,
                FPIOA.IIC1_SDA,
                oe=1,
                ie=1,
                pu=1,
                st=1,
                ds=15,
            )

            self.i2c = I2C(1, scl=34, sda=35, freq=40000)
            self.ready = True
            print("INA226 IIC1 init ok")

        except Exception as e:
            self.ready = False
            self.error_text = "IIC init error"
            print("INA226 IIC1 init failed:", e)

    def write_reg16(self, reg, value):
        value = int(value) & 0xFFFF
        data = bytes([
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        self.i2c.writeto_mem(INA226_ADDR, reg, data)

    def read_reg16_u(self, reg):
        data = self.i2c.readfrom_mem(INA226_ADDR, reg, 2)
        return (int(data[0]) << 8) | int(data[1])

    def read_reg16_s(self, reg):
        value = self.read_reg16_u(reg)
        if value & 0x8000:
            value -= 65536
        return value

    def init_ina226(self):
        self.write_reg16(REG_CONFIG, 0x4127)
        self.write_reg16(REG_CALIBRATION, CAL_VALUE)

    def read_current_by_shunt(self):
        raw_shunt = self.read_reg16_s(REG_SHUNT_VOLTAGE)
        shunt_v = raw_shunt * SHUNT_LSB_V
        shunt_mv = shunt_v * 1000.0
        current_a = shunt_v / RSHUNT

        if current_a < 0:
            current_a = -current_a

        return current_a * 1000.0, shunt_mv, raw_shunt

    def read_current_by_current_reg(self):
        self.write_reg16(REG_CALIBRATION, CAL_VALUE)
        raw_current = self.read_reg16_s(REG_CURRENT)
        current_a = raw_current * CURRENT_LSB

        if current_a < 0:
            current_a = -current_a

        return current_a * 1000.0, raw_current

    def read_bus_voltage(self):
        raw_bus = self.read_reg16_u(REG_BUS_VOLTAGE)
        bus_v = raw_bus * 1.25 / 1000.0
        return bus_v, raw_bus

    def scan_devices(self):
        try:
            self.devices = self.i2c.scan()
        except Exception:
            self.devices = []

        self.ina_found = INA226_ADDR in self.devices

        if not self.ina_found:
            self.init_ok = False
            self.smooth_current_shunt = None
            self.smooth_current_reg = None

    def update(self):
        if not self.ready:
            return

        now = time.ticks_ms()

        if time.ticks_diff(now, self.last_scan_ms) > 1000:
            self.last_scan_ms = now
            self.scan_devices()

        if not self.ina_found:
            return

        try:
            if not self.init_ok:
                self.init_ina226()
                self.init_ok = True

            current_shunt_mA, shunt_mV, raw_shunt = self.read_current_by_shunt()
            current_reg_mA, raw_current = self.read_current_by_current_reg()

            if self.smooth_current_shunt is None:
                self.smooth_current_shunt = current_shunt_mA
            else:
                self.smooth_current_shunt = (
                    self.smooth_current_shunt * (1.0 - self.alpha)
                    + current_shunt_mA * self.alpha
                )

            if self.smooth_current_reg is None:
                self.smooth_current_reg = current_reg_mA
            else:
                self.smooth_current_reg = (
                    self.smooth_current_reg * (1.0 - self.alpha)
                    + current_reg_mA * self.alpha
                )

            self.current_mA = self.smooth_current_shunt
            self.current_reg_mA = self.smooth_current_reg
            self.power_W = (self.current_mA / 1000.0) * 5.0
            self.shunt_mV = shunt_mV
            self.raw_shunt = raw_shunt

            try:
                self.bus_V, self.raw_bus = self.read_bus_voltage()
            except Exception:
                self.bus_V = 0.0
                self.raw_bus = 0

            self.error_text = ""

        except Exception as e:
            self.init_ok = False
            self.error_text = str(e)

    def draw_overlay(self, pl):
        x = 405
        y = 10

        if not self.ready:
            pl.osd_img.draw_string_advanced(
                x,
                y,
                18,
                "INA226 IIC ERR",
                color=(255, 255, 0, 0),
            )
            return

        if not self.ina_found:
            pl.osd_img.draw_string_advanced(
                x,
                y,
                18,
                "INA226 NOT FOUND",
                color=(255, 255, 0, 0),
            )
            return

        if self.error_text:
            pl.osd_img.draw_string_advanced(
                x,
                y,
                18,
                "INA226 READ ERR",
                color=(255, 255, 0, 0),
            )
            return

        pl.osd_img.draw_string_advanced(
            x,
            y,
            18,
            "I: %.1f mA" % self.current_mA,
            color=(255, 255, 255, 255),
        )

        pl.osd_img.draw_string_advanced(
            x,
            y + 24,
            18,
            "P: %.3f W" % self.power_W,
            color=(255, 255, 255, 0),
        )

        pl.osd_img.draw_string_advanced(
            x,
            y + 48,
            18,
            "Bus: %.3f V" % self.bus_V,
            color=(255, 180, 220, 255),
        )

    def deinit(self):
        try:
            if self.i2c:
                self.i2c.deinit()
        except Exception:
            pass


# ==========================================================
# 触摸屏读取封装：修正版
# K230 TOUCH.read(1) 返回 tuple/list，里面第 0 个才是触摸点对象
# 正确读取方式：
# points = tp.read(1)
# if len(points):
#     pt = points[0]
#     x = pt.x
#     y = pt.y
#     event = pt.event
# ==========================================================
class TouchReader:
    def __init__(self, display_size):
        self.display_size = display_size
        self.tp = None
        self.ok = False

        self.last_pressed = False
        self.last_x = -1
        self.last_y = -1

        # 坐标修正模式：
        # 0：不变
        # 1：左右翻转
        # 2：上下翻转
        # 3：左右+上下翻转
        # 4：x/y 交换
        # 5：x/y 交换后左右翻转
        # 6：x/y 交换后上下翻转
        # 7：x/y 交换后左右+上下翻转
        self.coord_mode = 0

        try:
            from machine import TOUCH
            self.TOUCH = TOUCH
            self.tp = TOUCH(0)
            self.ok = True
            print("Touch init ok: TOUCH(0)")
        except Exception as e:
            print("Touch init failed:", e)
            self.tp = None
            self.ok = False

    def fix_xy(self, x, y):
        w = self.display_size[0]
        h = self.display_size[1]

        if self.coord_mode == 0:
            pass

        elif self.coord_mode == 1:
            x = w - 1 - x

        elif self.coord_mode == 2:
            y = h - 1 - y

        elif self.coord_mode == 3:
            x = w - 1 - x
            y = h - 1 - y

        elif self.coord_mode == 4:
            tmp = x
            x = y
            y = tmp

        elif self.coord_mode == 5:
            tmp = x
            x = y
            y = tmp
            x = w - 1 - x

        elif self.coord_mode == 6:
            tmp = x
            x = y
            y = tmp
            y = h - 1 - y

        elif self.coord_mode == 7:
            tmp = x
            x = y
            y = tmp
            x = w - 1 - x
            y = h - 1 - y

        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x >= w:
            x = w - 1
        if y >= h:
            y = h - 1

        return x, y

    def read_touch(self):
        if not self.ok or self.tp is None:
            return False, -1, -1, -1

        try:
            points = self.tp.read(1)

            if points is None:
                return False, -1, -1, -1

            if len(points) <= 0:
                return False, -1, -1, -1

            pt = points[0]

            x = int(pt.x)
            y = int(pt.y)
            event = int(pt.event)

            x, y = self.fix_xy(x, y)

            return True, x, y, event

        except Exception as e:
            return False, -1, -1, -1

    def get_click(self):
        pressed, x, y, event = self.read_touch()

        clicked = False
        cx = -1
        cy = -1

        if pressed:
            is_down = False

            try:
                if event == self.TOUCH.EVENT_DOWN:
                    is_down = True
            except Exception:
                pass

            # 有些固件 event=0 也代表有效按下
            if event == 0:
                is_down = True

            # 如果没有明确 DOWN，但从未按下变成按下，也触发一次
            if is_down or (not self.last_pressed):
                clicked = True
                cx = x
                cy = y
                print("Touch click:", cx, cy, "event:", event)

            self.last_pressed = True
            self.last_x = x
            self.last_y = y

        else:
            self.last_pressed = False

        return clicked, cx, cy


# ==========================================================
# UI 按钮
# ==========================================================
class UIButton:
    def __init__(self, x, y, w, h, text):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.text = text

    def hit(self, px, py):
        if px >= self.x and px <= self.x + self.w and py >= self.y and py <= self.y + self.h:
            return True
        return False

    def draw(self, osd, color=(255, 40, 120, 255), text_color=(255, 255, 255, 255)):
        osd.draw_rectangle(
            self.x,
            self.y,
            self.w,
            self.h,
            color=color,
            thickness=3,
        )

        osd.draw_string_advanced(
            self.x + 18,
            self.y + int(self.h / 2) - 14,
            24,
            self.text,
            color=text_color,
        )


# ==========================================================
# 主检测类
# ==========================================================
class SegmentationApp(AIBase):
    def __init__(
        self,
        kmodel_path,
        labels,
        model_input_size,
        confidence_threshold=0.10,
        nms_threshold=0.45,
        mask_threshold=0.45,
        rgb888p_size=[1280, 960],
        display_size=[640, 480],
        debug_mode=0,
    ):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.mask_threshold = mask_threshold

        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        self.masks = np.zeros((1, self.display_size[1], self.display_size[0], 4))

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

        # ======================================================
        # D-bottom_y 标定表
        # 单位：D 为 cm，bottom_y 为 640x480 显示坐标下的目标底部像素 y
        # 标定数据来自已测量的 D-像素位置对应关系。
        # ======================================================
        self.distance_calib = [
            (477, 50.0),
            (474, 51.0),
            (469, 52.0),
            (466, 53.0),
            (459, 54.0),
            (457, 55.0),
            (452, 56.0),
            (449, 57.0),
            (447, 58.0),
            (443, 59.0),
            (441, 60.0),
            (438, 61.0),
            (435, 62.0),
            (433, 63.0),
            (428, 64.0),
            (426, 65.0),
            (425.5, 66.0),
            (425, 67.0),
            (421, 68.0),
            (420, 69.0),
            (418, 70.0),
            (417, 71.0),
            (413, 72.0),
            (412, 73.0),
            (411, 74.0),
            (408, 75.0),
            (405, 76.0),
            (404, 77.0),
            (403, 78.0),
            (402, 79.0),
            (400, 80.0),
            (399, 81.0),
            (397, 82.0),
            (396, 83.0),
            (394, 84.0),
            (393, 85.0),
            (388, 86.0),
            (387, 87.0),
            (386, 88.0),
            (385, 89.0),
            (384.5, 90.0),
            (384, 91.0),
            (383, 92.0),
            (381, 93.0),
            (380.5, 94.0),
            (380, 95.0),
            (379, 96.0),
            (378, 97.0),
            (377.5, 98.0),
            (373, 99.0),
            (372.5, 100.0),
        ]

        # H = D * hpix / fy_pixel
        # 用户提供的 GC2093 内参对应 1280x960：fy = 1200.6879329729338
        # 当前结果绘制在 640x480 坐标系下，因此 fy 乘以 0.5。
        self.fy_pixel = 600.3439664864669

        #hansome校准
        self.HEIGHT_SCALE = 1.129
        #hhh

        # 液面滤波
        self.level_history = []
        self.max_history_len = 5
        self.last_level_y = None

        # 动态模式跟踪保持
        self.track_box = None
        self.track_class_id = -1
        self.track_score = 0.0
        self.track_parts = 0
        self.track_miss = 100
        self.track_hold_max = 8

        self.box_alpha = 0.28

        self.smooth_D = None
        self.smooth_H = None
        self.smooth_L = None
        self.smooth_level_y = None
        self.value_alpha = 0.25

        # 固定测量模式
        self.fixed_samples = []
        self.fixed_result = None
        self.fixed_measuring = False
        self.fixed_start_ms = 0
        self.fixed_measure_time_ms = 3000

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            top, bottom, left, right = self.get_padding_param()

            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [114, 114, 114],
            )

            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )

            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            seg_res = aidemo.segment_postprocess(
                results,
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                self.model_input_size,
                [self.display_size[1], self.display_size[0]],
                self.confidence_threshold,
                self.nms_threshold,
                self.mask_threshold,
                self.masks,
            )
            return seg_res

    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]

        ratio_w = float(dst_w) / self.rgb888p_size[0]
        ratio_h = float(dst_h) / self.rgb888p_size[1]
        ratio = ratio_w if ratio_w < ratio_h else ratio_h

        new_w = int(ratio * self.rgb888p_size[0])
        new_h = int(ratio * self.rgb888p_size[1])

        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2

        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        return top, bottom, left, right

    # ======================================================
    # 基础工具
    # ======================================================
    def ema_value(self, old, new, alpha):
        if new is None:
            return old
        if old is None:
            return new
        return old * (1.0 - alpha) + new * alpha

    def median_value(self, arr):
        if arr is None or len(arr) == 0:
            return None

        temp = []
        for v in arr:
            temp.append(v)

        temp.sort()
        return temp[len(temp) // 2]

    def abs_value(self, v):
        if v < 0:
            return -v
        return v

    def smooth_bbox(self, x1, y1, w, h):
        new_box = [float(x1), float(y1), float(w), float(h)]

        if self.track_box is None:
            self.track_box = new_box
        else:
            for i in range(4):
                self.track_box[i] = (
                    self.track_box[i] * (1.0 - self.box_alpha)
                    + new_box[i] * self.box_alpha
                )

        sx1 = int(self.track_box[0])
        sy1 = int(self.track_box[1])
        sw = int(self.track_box[2])
        sh = int(self.track_box[3])

        if sx1 < 0:
            sx1 = 0
        if sy1 < 0:
            sy1 = 0

        if sx1 + sw > self.display_size[0]:
            sw = self.display_size[0] - sx1
        if sy1 + sh > self.display_size[1]:
            sh = self.display_size[1] - sy1

        if sw < 1:
            sw = 1
        if sh < 1:
            sh = 1

        return sx1, sy1, sw, sh

    # ======================================================
    # D / H / L
    # ======================================================
    def estimate_distance(self, bottom_y):
        # 使用 bottom_y-D 标定表做分段线性插值。
        # D 的单位是 cm。
        if self.distance_calib is None or len(self.distance_calib) == 0:
            return 0

        y = float(bottom_y)

        # 标定表按像素 y 从小到大排序：y 越小通常代表目标越远。
        calib = []
        for p in self.distance_calib:
            calib.append((float(p[0]), float(p[1])))

        calib.sort(key=lambda p: p[0])

        # 超出标定范围时，使用边界值，避免输出离谱距离。
        if y <= calib[0][0]:
            return calib[0][1]

        if y >= calib[len(calib) - 1][0]:
            return calib[len(calib) - 1][1]

        for i in range(len(calib) - 1):
            y0, d0 = calib[i]
            y1, d1 = calib[i + 1]

            if y >= y0 and y <= y1:
                if y1 == y0:
                    return (d0 + d1) / 2.0

                t = (y - y0) / (y1 - y0)
                return d0 + t * (d1 - d0)

        return calib[len(calib) - 1][1]

    def estimate_height(self, D, hpix):
        if self.fy_pixel <= 0:
            return 0

        H = D * hpix / self.fy_pixel
        H = H * self.HEIGHT_SCALE  # hansome校准
        return H

    def estimate_liquid_height(self, H, bottom_y, level_y, hpix):
        if level_y is None:
            return None
        if hpix <= 0:
            return None

        lpix = bottom_y - level_y

        if lpix < 0:
            lpix = 0
        if lpix > hpix:
            lpix = hpix

        L = H * lpix / hpix
        L = L * self.HEIGHT_SCALE  # hansome校准
        return L

    # ======================================================
    # 坐标映射与像素读取
    # ======================================================
    def display_to_input_roi(self, x, y, w, h):
        sx = float(self.rgb888p_size[0]) / float(self.display_size[0])
        sy = float(self.rgb888p_size[1]) / float(self.display_size[1])

        rx = int(x * sx)
        ry = int(y * sy)
        rw = int(w * sx)
        rh = int(h * sy)

        if rx < 0:
            rx = 0
        if ry < 0:
            ry = 0

        if rx >= self.rgb888p_size[0]:
            rx = self.rgb888p_size[0] - 1
        if ry >= self.rgb888p_size[1]:
            ry = self.rgb888p_size[1] - 1

        if rx + rw > self.rgb888p_size[0]:
            rw = self.rgb888p_size[0] - rx
        if ry + rh > self.rgb888p_size[1]:
            rh = self.rgb888p_size[1] - ry

        if rw < 1:
            rw = 1
        if rh < 1:
            rh = 1

        return rx, ry, rw, rh

    def get_rgb_from_ndarray(self, img, x, y):
        try:
            shape_len = len(img.shape)

            if shape_len == 4:
                r = int(img[0][0][y][x])
                g = int(img[0][1][y][x])
                b = int(img[0][2][y][x])
            elif shape_len == 3:
                r = int(img[0][y][x])
                g = int(img[1][y][x])
                b = int(img[2][y][x])
            else:
                r, g, b = 0, 0, 0

            return r, g, b

        except Exception:
            return 0, 0, 0

    # ======================================================
    # 液面滤波
    # ======================================================
    def reset_level_filter(self):
        self.level_history = []
        self.last_level_y = None
        self.smooth_level_y = None

    def median_filter_level(self, y):
        if y is None:
            return None

        if self.last_level_y is not None:
            if y - self.last_level_y > 35:
                y = self.last_level_y + 35
            elif self.last_level_y - y > 35:
                y = self.last_level_y - 35

        self.level_history.append(y)

        if len(self.level_history) > self.max_history_len:
            self.level_history.pop(0)

        temp = []
        for v in self.level_history:
            temp.append(v)

        temp.sort()
        mid = len(temp) // 2
        y_med = temp[mid]

        self.last_level_y = y_med

        return y_med

    # ======================================================
    # 液面检测：多列扫描 + 水平投票
    # ======================================================
    def detect_liquid_from_ndarray(self, img, x1, y1, w, h):
        roi_x = x1 + int(w * 0.22)
        roi_y = y1 + int(h * 0.18)
        roi_w = int(w * 0.56)
        roi_h = int(h * 0.68)

        rx, ry, rw, rh = self.display_to_input_roi(roi_x, roi_y, roi_w, roi_h)

        if rw <= 8 or rh <= 12:
            return None, "Unknown", 0

        num_cols = 9
        bin_size = 2
        grad_th = 5
        min_votes = 3

        candidates = []

        for ci in range(num_cols):
            if num_cols == 1:
                xx = rx + rw // 2
            else:
                xx = rx + int((ci + 1) * rw / (num_cols + 1))

            profile = []

            for yy in range(ry, ry + rh):
                total = 0
                cnt = 0

                for dx in [-1, 0, 1]:
                    px = xx + dx

                    if px < rx or px >= rx + rw:
                        continue

                    r, g, b = self.get_rgb_from_ndarray(img, px, yy)

                    luma = (r * 30 + g * 59 + b * 11) // 100
                    max_c = max(r, g, b)
                    min_c = min(r, g, b)
                    chroma = max_c - min_c

                    feature = luma + chroma // 3

                    total += feature
                    cnt += 1

                if cnt > 0:
                    profile.append(total // cnt)
                else:
                    profile.append(0)

            n = len(profile)

            if n < 10:
                continue

            smooth = []

            for i in range(n):
                if i == 0:
                    v = (profile[i] + profile[i + 1]) // 2
                elif i == n - 1:
                    v = (profile[i - 1] + profile[i]) // 2
                else:
                    v = (profile[i - 1] + profile[i] + profile[i + 1]) // 3

                smooth.append(v)

            start_i = int(n * 0.12)
            end_i = int(n * 0.88)

            local_peaks = []

            for i in range(start_i + 2, end_i - 2):
                g1 = smooth[i] - smooth[i - 1]
                if g1 < 0:
                    g1 = -g1

                g2 = smooth[i + 1] - smooth[i]
                if g2 < 0:
                    g2 = -g2

                grad = g1 + g2

                if grad < grad_th:
                    continue

                grad_left = smooth[i - 1] - smooth[i - 2]
                if grad_left < 0:
                    grad_left = -grad_left

                grad_right = smooth[i + 2] - smooth[i + 1]
                if grad_right < 0:
                    grad_right = -grad_right

                if grad >= grad_left and grad >= grad_right:
                    y_input = ry + i
                    local_peaks.append((y_input, grad))

            for a in range(len(local_peaks)):
                for b in range(a + 1, len(local_peaks)):
                    if local_peaks[b][1] > local_peaks[a][1]:
                        tmp = local_peaks[a]
                        local_peaks[a] = local_peaks[b]
                        local_peaks[b] = tmp

            keep_num = 3
            if len(local_peaks) < keep_num:
                keep_num = len(local_peaks)

            for k in range(keep_num):
                candidates.append(local_peaks[k])

        if len(candidates) == 0:
            return None, "Unknown", 0

        bins_y = []
        bins_vote = []
        bins_strength = []

        for y_input, grad in candidates:
            y_bin = (y_input // bin_size) * bin_size

            found = False

            for bi in range(len(bins_y)):
                if bins_y[bi] == y_bin:
                    bins_vote[bi] += 1
                    bins_strength[bi] += grad
                    found = True
                    break

            if not found:
                bins_y.append(y_bin)
                bins_vote.append(1)
                bins_strength.append(grad)

        best_bin = -1
        best_score = -1

        for bi in range(len(bins_y)):
            vote = bins_vote[bi]
            strength = bins_strength[bi]

            if vote < min_votes:
                continue

            score = vote * 100 + strength

            if score > best_score:
                best_score = score
                best_bin = bi

        if best_bin < 0:
            return None, "Unknown", 0

        liquid_y_input = bins_y[best_bin]

        sy_inv = float(self.display_size[1]) / float(self.rgb888p_size[1])
        liquid_y_display = int(liquid_y_input * sy_inv)

        liquid_y_display = self.median_filter_level(liquid_y_display)

        bottom_y_display = y1 + h
        level_px_display = bottom_y_display - liquid_y_display

        if level_px_display < 0:
            level_px_display = 0

        liquid_type = self.classify_liquid_color(
            img,
            rx,
            liquid_y_input + 4,
            rw,
            ry + rh,
        )

        return liquid_y_display, liquid_type, level_px_display

    def classify_liquid_color(self, img, rx, y_start, rw, y_end):
        if y_start >= y_end:
            return "Unknown"

        x_step = 3
        y_step = 3

        sum_r = 0
        sum_g = 0
        sum_b = 0
        cnt = 0

        for yy in range(y_start, y_end, y_step):
            for xx in range(rx, rx + rw, x_step):
                r, g, b = self.get_rgb_from_ndarray(img, xx, yy)

                sum_r += r
                sum_g += g
                sum_b += b
                cnt += 1

        if cnt <= 0:
            return "Unknown"

        r = sum_r // cnt
        g = sum_g // cnt
        b = sum_b // cnt

        brightness = (r + g + b) // 3
        max_c = max(r, g, b)
        min_c = min(r, g, b)
        saturation_like = max_c - min_c

        if brightness < 55:
            return "Cola/Tea"

        if brightness > 165 and saturation_like < 35:
            return "Milk/White"

        if brightness > 115 and saturation_like < 25:
            return "Water"

        if r > g and g > b and r > 90:
            return "Orange"

        if g > r and g > b and saturation_like > 35:
            return "Green"

        if r > g and r > b and saturation_like > 35:
            return "Red"

        return "Unknown"

    # ======================================================
    # 轴线碎片融合
    # ======================================================
    def judge_shape_class(self, merged_w, merged_h, has_bottle):
        if merged_w <= 0:
            return 41

        aspect = float(merged_h) / float(merged_w)

        if aspect > 1.55:
            return 39

        if has_bottle and aspect > 1.25:
            return 39

        return 41

    def get_merged_axis_target(self, dets, ids, scores):
        axis_x = self.display_size[0] // 2

        candidates = []
        max_score = 0.0
        has_bottle = False

        for i, det in enumerate(dets):
            class_id = int(ids[i])
            score = float(scores[i])

            # COCO: bottle = 39, cup = 41
            if class_id not in [39, 41]:
                continue

            if score < 0.08:
                continue

            x1, y1, w, h = map(lambda x: int(round(x, 0)), det)

            if w <= 0 or h <= 0:
                continue

            if x1 < 0:
                x1 = 0
            if y1 < 0:
                y1 = 0

            if x1 + w > self.display_size[0]:
                w = self.display_size[0] - x1
            if y1 + h > self.display_size[1]:
                h = self.display_size[1] - y1

            if w <= 0 or h <= 0:
                continue

            area = w * h
            if area < 450:
                continue

            cx = x1 + w // 2
            axis_error = abs(cx - axis_x)

            if axis_error > 130:
                continue

            if w > int(self.display_size[0] * 0.60):
                continue

            aspect = float(h) / float(w)
            if aspect < 0.40:
                continue

            candidates.append({
                "x1": x1,
                "y1": y1,
                "x2": x1 + w,
                "y2": y1 + h,
                "w": w,
                "h": h,
                "cx": cx,
                "class_id": class_id,
                "score": score,
            })

            if class_id == 39:
                has_bottle = True

            if score > max_score:
                max_score = score

        if len(candidates) == 0:
            return None

        main_idx = 0
        main_area = -1

        for i in range(len(candidates)):
            area = candidates[i]["w"] * candidates[i]["h"]
            if area > main_area:
                main_area = area
                main_idx = i

        main_box = candidates[main_idx]
        main_cx = main_box["cx"]
        main_w = main_box["w"]

        selected = []

        for c in candidates:
            if abs(c["cx"] - main_cx) > max(60, int(main_w * 1.3)):
                continue

            overlap_x1 = max(c["x1"], main_box["x1"])
            overlap_x2 = min(c["x2"], main_box["x2"])
            overlap_w = overlap_x2 - overlap_x1

            min_w = c["w"] if c["w"] < main_box["w"] else main_box["w"]

            if overlap_w < int(min_w * 0.20):
                continue

            selected.append(c)

        if len(selected) == 0:
            selected = [main_box]

        min_x = selected[0]["x1"]
        min_y = selected[0]["y1"]
        max_x = selected[0]["x2"]
        max_y = selected[0]["y2"]

        for c in selected:
            if c["x1"] < min_x:
                min_x = c["x1"]
            if c["y1"] < min_y:
                min_y = c["y1"]
            if c["x2"] > max_x:
                max_x = c["x2"]
            if c["y2"] > max_y:
                max_y = c["y2"]

        merged_w = max_x - min_x
        merged_h = max_y - min_y

        if merged_w <= 0 or merged_h <= 0:
            return None

        if merged_h < 60:
            return None

        if merged_w > int(self.display_size[0] * 0.65):
            return None

        merged_class_id = self.judge_shape_class(merged_w, merged_h, has_bottle)

        return {
            "box": [min_x, min_y, merged_w, merged_h],
            "class_id": merged_class_id,
            "score": max_score,
            "parts": len(selected),
        }

    # ======================================================
    # 动态模式稳定目标
    # ======================================================
    def get_stable_fused_target(self, seg_res):
        if seg_res is None or not seg_res[0]:
            if self.track_box is not None and self.track_miss < self.track_hold_max:
                self.track_miss += 1
                return self.track_box, self.track_class_id, self.track_score, self.track_parts, True
            return None, -1, 0.0, 0, False

        dets, ids, scores = seg_res[0], seg_res[1], seg_res[2]
        merged_target = self.get_merged_axis_target(dets, ids, scores)

        if merged_target is None:
            if self.track_box is not None and self.track_miss < self.track_hold_max:
                self.track_miss += 1
                return self.track_box, self.track_class_id, self.track_score, self.track_parts, True
            return None, -1, 0.0, 0, False

        x1, y1, w, h = merged_target["box"]
        class_id = merged_target["class_id"]
        score = merged_target["score"]
        parts = merged_target["parts"]

        sx1, sy1, sw, sh = self.smooth_bbox(x1, y1, w, h)

        self.track_box = [sx1, sy1, sw, sh]
        self.track_class_id = class_id
        self.track_score = score
        self.track_parts = parts
        self.track_miss = 0

        return self.track_box, class_id, score, parts, False

    # ======================================================
    # 计算一帧结果
    # ======================================================
    def calc_one_result(self, img, seg_res, use_stable):
        if use_stable:
            target_box, class_id, score, parts, is_hold = self.get_stable_fused_target(seg_res)
        else:
            if seg_res is None or not seg_res[0]:
                return None

            dets, ids, scores = seg_res[0], seg_res[1], seg_res[2]
            mt = self.get_merged_axis_target(dets, ids, scores)

            if mt is None:
                return None

            target_box = mt["box"]
            class_id = mt["class_id"]
            score = mt["score"]
            parts = mt["parts"]
            is_hold = False

        if target_box is None:
            return None

        x1 = int(target_box[0])
        y1 = int(target_box[1])
        w = int(target_box[2])
        h = int(target_box[3])

        if x1 < 0:
            x1 = 0
        if y1 < 0:
            y1 = 0

        if x1 + w > self.display_size[0]:
            w = self.display_size[0] - x1
        if y1 + h > self.display_size[1]:
            h = self.display_size[1] - y1

        if w <= 0 or h <= 0:
            return None

        bottom_x = int(x1 + w / 2)
        bottom_y = int(y1 + h)
        hpix = h

        D = self.estimate_distance(bottom_y)
        H = self.estimate_height(D, hpix)

        level_y, liquid_type, liquid_level_px = self.detect_liquid_from_ndarray(
            img,
            x1,
            y1,
            w,
            h,
        )

        L = self.estimate_liquid_height(H, bottom_y, level_y, hpix)

        return {
            "x1": x1,
            "y1": y1,
            "w": w,
            "h": h,
            "class_id": class_id,
            "score": score,
            "parts": parts,
            "hold": is_hold,
            "bottom_x": bottom_x,
            "bottom_y": bottom_y,
            "D": D,
            "H": H,
            "L": L,
            "level_y": level_y,
            "liquid_type": liquid_type,
            "liquid_level_px": liquid_level_px,
        }

    # ======================================================
    # 动态检测模式绘制
    # ======================================================
    def draw_dynamic_result(self, pl, img, seg_res):
        result = self.calc_one_result(img, seg_res, True)

        axis_x = self.display_size[0] // 2
        pl.osd_img.draw_line(
            axis_x,
            0,
            axis_x,
            self.display_size[1],
            color=(120, 255, 255, 255),
            thickness=1,
        )

        if result is None:
            pl.osd_img.draw_string_advanced(
                20,
                20,
                24,
                "Dynamic: No valid target",
                color=(255, 255, 255, 255),
            )
            return

        self.smooth_D = self.ema_value(self.smooth_D, result["D"], self.value_alpha)
        self.smooth_H = self.ema_value(self.smooth_H, result["H"], self.value_alpha)

        if result["L"] is not None:
            self.smooth_L = self.ema_value(self.smooth_L, result["L"], self.value_alpha)

        if result["level_y"] is not None:
            self.smooth_level_y = self.ema_value(
                self.smooth_level_y,
                result["level_y"],
                self.value_alpha,
            )
            result["level_y"] = int(self.smooth_level_y)

        result["D"] = self.smooth_D
        result["H"] = self.smooth_H
        result["L"] = self.smooth_L

        self.draw_measure_result(pl, result, "Dynamic")

    # ======================================================
    # 固定测量模式
    # ======================================================
    def start_fixed_measurement(self):
        self.fixed_samples = []
        self.fixed_result = None
        self.fixed_measuring = True
        self.fixed_start_ms = time.ticks_ms()

        self.reset_level_filter()

        print("Fixed measurement start")

    def process_fixed_frame(self, img, seg_res):
        if not self.fixed_measuring:
            return

        result = self.calc_one_result(img, seg_res, False)

        if result is not None:
            if result["w"] > 10 and result["h"] > 40:
                self.fixed_samples.append(result)

        now = time.ticks_ms()
        if time.ticks_diff(now, self.fixed_start_ms) >= self.fixed_measure_time_ms:
            self.finish_fixed_measurement()

    def finish_fixed_measurement(self):
        self.fixed_measuring = False

        if len(self.fixed_samples) == 0:
            self.fixed_result = None
            print("Fixed measurement failed: no samples")
            return

        bottom_list = []
        h_list = []
        d_list = []

        for s in self.fixed_samples:
            bottom_list.append(s["bottom_y"])
            h_list.append(s["h"])
            d_list.append(s["D"])

        med_bottom = self.median_value(bottom_list)
        med_h = self.median_value(h_list)
        med_D = self.median_value(d_list)

        useful = []

        for s in self.fixed_samples:
            ok = True

            if med_bottom is not None:
                if self.abs_value(s["bottom_y"] - med_bottom) > 25:
                    ok = False

            if med_h is not None:
                if self.abs_value(s["h"] - med_h) > 35:
                    ok = False

            if med_D is not None:
                if self.abs_value(s["D"] - med_D) > 12:
                    ok = False

            if ok:
                useful.append(s)

        if len(useful) == 0:
            useful = self.fixed_samples

        xs = []
        ys = []
        ws = []
        hs = []
        bxs = []
        bys = []
        Ds = []
        Hs = []
        Ls = []
        levels = []
        scores = []
        parts_list = []

        for s in useful:
            xs.append(s["x1"])
            ys.append(s["y1"])
            ws.append(s["w"])
            hs.append(s["h"])
            bxs.append(s["bottom_x"])
            bys.append(s["bottom_y"])
            Ds.append(s["D"])
            Hs.append(s["H"])
            scores.append(s["score"])
            parts_list.append(s["parts"])

            if s["L"] is not None:
                Ls.append(s["L"])

            if s["level_y"] is not None:
                levels.append(s["level_y"])

        x1 = int(self.median_value(xs))
        y1 = int(self.median_value(ys))
        w = int(self.median_value(ws))
        h = int(self.median_value(hs))
        bottom_x = int(self.median_value(bxs))
        bottom_y = int(self.median_value(bys))
        D = self.median_value(Ds)
        H = self.median_value(Hs)

        if len(Ls) > 0:
            L = self.median_value(Ls)
        else:
            L = None

        if len(levels) > 0:
            level_y = int(self.median_value(levels))
        else:
            level_y = None

        self.fixed_result = {
            "x1": x1,
            "y1": y1,
            "w": w,
            "h": h,
            "class_id": 39,
            "score": self.median_value(scores),
            "parts": int(self.median_value(parts_list)),
            "hold": False,
            "bottom_x": bottom_x,
            "bottom_y": bottom_y,
            "D": D,
            "H": H,
            "L": L,
            "level_y": level_y,
            "liquid_type": "Fixed",
            "liquid_level_px": bottom_y - level_y if level_y is not None else 0,
            "valid_count": len(useful),
            "total_count": len(self.fixed_samples),
        }

        print("Fixed measurement done")
        print("total:", len(self.fixed_samples), "useful:", len(useful))

    # ======================================================
    # 通用绘制测量结果
    # ======================================================
    def draw_measure_result(self, pl, r, title):
        x1 = int(r["x1"])
        y1 = int(r["y1"])
        w = int(r["w"])
        h = int(r["h"])

        class_id = int(r["class_id"])
        score = float(r["score"])
        parts = int(r["parts"])

        if class_id == 39:
            show_name = "bottleF"
            box_color = (255, 0, 255, 0)
        else:
            show_name = "cupF"
            box_color = (255, 255, 180, 0)

        if r["hold"]:
            show_name = show_name + "_hold"
            box_color = (255, 255, 255, 0)

        pl.osd_img.draw_rectangle(
            x1,
            y1,
            w,
            h,
            color=box_color,
            thickness=3,
        )

        pl.osd_img.draw_string_advanced(
            x1,
            max(0, y1 - 28),
            22,
            "%s %.2f P:%d" % (show_name, score, parts),
            color=box_color,
        )

        bottom_x = int(r["bottom_x"])
        bottom_y = int(r["bottom_y"])

        pl.osd_img.draw_circle(
            bottom_x,
            bottom_y,
            6,
            color=(255, 255, 0, 0),
            fill=True,
        )

        pl.osd_img.draw_string_advanced(
            bottom_x - 80,
            min(self.display_size[1] - 25, bottom_y + 8),
            22,
            "B:(%d,%d)" % (bottom_x, bottom_y),
            color=(255, 255, 0, 0),
        )

        pl.osd_img.draw_string_advanced(
            x1,
            y1 + 5,
            22,
            "D:%.1fcm" % r["D"],
            color=(255, 255, 255, 0),
        )

        pl.osd_img.draw_string_advanced(
            x1,
            y1 + 32,
            22,
            "H:%.1fcm" % r["H"],
            color=(255, 255, 255, 0),
        )

        if r["L"] is not None:
            pl.osd_img.draw_string_advanced(
                x1,
                y1 + 59,
                22,
                "L:%.1fcm" % r["L"],
                color=(255, 0, 191, 255),
            )
        else:
            pl.osd_img.draw_string_advanced(
                x1,
                y1 + 59,
                22,
                "L:None",
                color=(255, 0, 191, 255),
            )

        if r["level_y"] is not None:
            level_y = int(r["level_y"])
            line_x1 = x1 + int(w * 0.12)
            line_x2 = x1 + int(w * 0.88)

            pl.osd_img.draw_line(
                line_x1,
                level_y,
                line_x2,
                level_y,
                color=(255, 0, 191, 255),
                thickness=3,
            )

        pl.osd_img.draw_string_advanced(
            20,
            18,
            24,
            title,
            color=(255, 255, 255, 255),
        )

        if "valid_count" in r:
            pl.osd_img.draw_string_advanced(
                20,
                48,
                22,
                "Samples:%d/%d" % (r["valid_count"], r["total_count"]),
                color=(255, 255, 255, 255),
            )


# ==========================================================
# UI 系统
# ==========================================================
class MeasureUI:
    MODE_HOME = 0
    MODE_DYNAMIC = 1
    MODE_FIXED = 2

    def __init__(self, display_size):
        self.display_size = display_size
        self.mode = self.MODE_HOME

        self.btn_dynamic = UIButton(90, 145, 460, 70, "Dynamic Detect")
        self.btn_fixed = UIButton(90, 250, 460, 70, "Fixed 3s Measure")

        self.btn_home = UIButton(10, 420, 150, 50, "Home")
        self.btn_start = UIButton(180, 420, 210, 50, "Start")
        self.btn_remeasure = UIButton(400, 420, 220, 50, "Remeasure")

    def draw_home(self, pl):
        pl.osd_img.clear()

        pl.osd_img.draw_string_advanced(
            120,
            55,
            34,
            "K230 Measure 1280",
            color=(255, 255, 255, 255),
        )

        pl.osd_img.draw_string_advanced(
            145,
            100,
            24,
            "Touch to select mode",
            color=(255, 180, 220, 255),
        )

        self.btn_dynamic.draw(pl.osd_img, color=(255, 40, 160, 255))
        self.btn_fixed.draw(pl.osd_img, color=(255, 40, 220, 120))

        pl.osd_img.draw_string_advanced(
            80,
            355,
            20,
            "Dynamic: real-time detection",
            color=(255, 255, 255, 255),
        )

        pl.osd_img.draw_string_advanced(
            80,
            385,
            20,
            "Fixed: 3s multi-sampling and locked result",
            color=(255, 255, 255, 255),
        )

    def draw_dynamic_ui(self, pl):
        self.btn_home.draw(pl.osd_img, color=(255, 200, 80, 80))

    def draw_fixed_ui(self, pl, seg):
        self.btn_home.draw(pl.osd_img, color=(255, 200, 80, 80))

        if seg.fixed_measuring:
            pl.osd_img.draw_string_advanced(
                190,
                430,
                24,
                "Measuring 3s...",
                color=(255, 255, 255, 255),
            )
        else:
            self.btn_start.draw(pl.osd_img, color=(255, 80, 180, 80))
            self.btn_remeasure.draw(pl.osd_img, color=(255, 80, 120, 220))

    def handle_touch(self, x, y, seg):
        print("UI handle touch:", x, y, "mode:", self.mode)

        if self.mode == self.MODE_HOME:
            if self.btn_dynamic.hit(x, y):
                self.mode = self.MODE_DYNAMIC
                print("Enter dynamic mode")
                return

            if self.btn_fixed.hit(x, y):
                self.mode = self.MODE_FIXED
                print("Enter fixed mode")
                return

        elif self.mode == self.MODE_DYNAMIC:
            if self.btn_home.hit(x, y):
                self.mode = self.MODE_HOME
                print("Back home")
                return

        elif self.mode == self.MODE_FIXED:
            if self.btn_home.hit(x, y):
                self.mode = self.MODE_HOME
                print("Back home")
                return

            if not seg.fixed_measuring:
                if self.btn_start.hit(x, y) or self.btn_remeasure.hit(x, y):
                    seg.start_fixed_measurement()
                    return


# ==========================================================
# 主函数
# ==========================================================
if __name__ == "__main__":
    display_mode = "lcd"

    if display_mode == "hdmi":
        display_size = [1920, 1080]
    else:
        display_size = [640, 480]

    kmodel_path = "/sdcard/kmodel/yolov8n_seg_320.kmodel"

    labels = [
        "person", "bicycle", "car", "motorcycle", "airplane", "bus",
        "train", "truck", "boat", "traffic light", "fire hydrant",
        "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
        "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush",
    ]

    confidence_threshold = 0.10
    nms_threshold = 0.45
    mask_threshold = 0.45

    # 关键修改：采集/AI输入源尺寸改为 1280x960
    # 注意：kmodel 仍是 320x320，config_preprocess() 会自动 resize 到 model_input_size=[320, 320]
    rgb888p_size = [1280, 960]

    pl = None
    seg = None
    touch = None
    power_monitor = None

    try:
        key = YbKey()
        touch = TouchReader(display_size)
        ui = MeasureUI(display_size)
        power_monitor = INA226Monitor()

        pl = PipeLine(
            rgb888p_size=rgb888p_size,
            display_size=display_size,
            display_mode=display_mode,
        )

        pl.create()

        seg = SegmentationApp(
            kmodel_path,
            labels=labels,
            model_input_size=[320, 320],
            confidence_threshold=confidence_threshold,
            nms_threshold=nms_threshold,
            mask_threshold=mask_threshold,
            rgb888p_size=rgb888p_size,
            display_size=display_size,
            debug_mode=0,
        )

        seg.config_preprocess()

        print("Camera/RGB input size:", rgb888p_size)
        print("Model input size: [320, 320]")
        print("Display/UI size:", display_size)

        while True:
            with ScopedTiming("total", 1):
                clicked, tx, ty = touch.get_click()

                if clicked:
                    print("UI touch:", tx, ty)
                    ui.handle_touch(tx, ty, seg)

                # 按键兜底：
                # 主页按键进入固定测量；
                # 固定模式按键开始/重新测量。
                if key.is_pressed():
                    if ui.mode == ui.MODE_HOME:
                        ui.mode = ui.MODE_FIXED
                    elif ui.mode == ui.MODE_FIXED and not seg.fixed_measuring:
                        seg.start_fixed_measurement()
                    time.sleep_ms(300)

                power_monitor.update()
                img = pl.get_frame()

                # 主页不跑 AI，节省算力
                if ui.mode == ui.MODE_HOME:
                    ui.draw_home(pl)
                    power_monitor.draw_overlay(pl)
                    pl.show_image()
                    gc.collect()
                    continue

                seg_res = seg.run(img)
                pl.osd_img.clear()

                if ui.mode == ui.MODE_DYNAMIC:
                    seg.draw_dynamic_result(pl, img, seg_res)
                    ui.draw_dynamic_ui(pl)

                elif ui.mode == ui.MODE_FIXED:
                    seg.process_fixed_frame(img, seg_res)

                    if seg.fixed_measuring:
                        current = seg.calc_one_result(img, seg_res, False)

                        if current is not None:
                            seg.draw_measure_result(pl, current, "Fixed Measuring")
                        else:
                            pl.osd_img.draw_string_advanced(
                                20,
                                30,
                                24,
                                "Fixed Measuring: No valid target",
                                color=(255, 255, 255, 255),
                            )

                        pl.osd_img.draw_string_advanced(
                            20,
                            65,
                            22,
                            "Samples:%d" % len(seg.fixed_samples),
                            color=(255, 255, 255, 255),
                        )

                    else:
                        if seg.fixed_result is not None:
                            seg.draw_measure_result(pl, seg.fixed_result, "Fixed Result")
                        else:
                            pl.osd_img.draw_string_advanced(
                                30,
                                70,
                                28,
                                "Fixed Mode",
                                color=(255, 255, 255, 255),
                            )

                            pl.osd_img.draw_string_advanced(
                                30,
                                115,
                                22,
                                "Place target on axis, then touch Start.",
                                color=(255, 255, 255, 255),
                            )

                            pl.osd_img.draw_string_advanced(
                                30,
                                150,
                                22,
                                "It will sample for 3 seconds.",
                                color=(255, 255, 255, 255),
                            )

                            pl.osd_img.draw_string_advanced(
                                30,
                                185,
                                22,
                                "Final result will be locked.",
                                color=(255, 255, 255, 255),
                            )

                    ui.draw_fixed_ui(pl, seg)

                power_monitor.draw_overlay(pl)
                pl.show_image()
                gc.collect()

    except KeyboardInterrupt:
        print("Program stopped by user")

    except Exception as e:
        print("Program error:", e)

    finally:
        try:
            if seg:
                seg.deinit()
        except Exception as e:
            print("seg deinit error:", e)

        try:
            if pl:
                pl.destroy()
        except Exception as e:
            print("pipeline destroy error:", e)

        try:
            if touch and touch.tp:
                touch.tp.deinit()
        except Exception:
            pass

        try:
            if power_monitor:
                power_monitor.deinit()
        except Exception:
            pass

        gc.collect()
