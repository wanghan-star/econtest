from libs.PipeLine import PipeLine
from machine import I2C, FPIOA
import time
import gc

# ==========================================================
# K230 + INA226 电流/功率 LCD 显示测试程序
# 使用 GPIO34 / GPIO35 / IIC1
#
# 接线：
# PCB SCL -> K230 GPIO34 / IIC1_SCL
# PCB SDA -> K230 GPIO35 / IIC1_SDA
# PCB GND -> K230 GND
# PCB 3V3 -> K230 3.3V
#
# 采样电阻：
# RSHUNT = 0.02Ω
#
# 推荐电流算法：
# I = Vshunt / Rshunt
# ==========================================================

INA226_ADDR = 0x40

REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

# 你的采样电阻
RSHUNT = 0.0436

# INA226 Shunt Voltage Register LSB = 2.5uV
SHUNT_LSB_V = 2.5e-6

# Current Register 的 LSB
CURRENT_LSB = 0.0001  # A/bit，0.1mA/bit

# Calibration = 0.00512 / (Current_LSB * Rshunt)
CAL_VALUE = int(0.00512 / (CURRENT_LSB * RSHUNT))


def write_reg16(i2c, addr, reg, value):
    value = int(value) & 0xFFFF
    data = bytes([
        (value >> 8) & 0xFF,
        value & 0xFF
    ])
    i2c.writeto_mem(addr, reg, data)


def read_reg16_u(i2c, addr, reg):
    data = i2c.readfrom_mem(addr, reg, 2)
    value = (int(data[0]) << 8) | int(data[1])
    return value


def read_reg16_s(i2c, addr, reg):
    value = read_reg16_u(i2c, addr, reg)
    if value & 0x8000:
        value -= 65536
    return value


def init_ina226(i2c):
    # 连续测量模式
    write_reg16(i2c, INA226_ADDR, REG_CONFIG, 0x4127)

    # 写入校准寄存器
    write_reg16(i2c, INA226_ADDR, REG_CALIBRATION, CAL_VALUE)


def read_current_by_shunt(i2c):
    """
    最推荐的调试算法：
    直接读取 INA226 的 Shunt Voltage Register
    I = Vshunt / Rshunt
    """
    raw_shunt = read_reg16_s(i2c, INA226_ADDR, REG_SHUNT_VOLTAGE)

    shunt_V = raw_shunt * SHUNT_LSB_V
    shunt_mV = shunt_V * 1000.0

    current_A = shunt_V / RSHUNT

    if current_A < 0:
        current_A = -current_A

    current_mA = current_A * 1000.0

    return current_mA, shunt_mV, raw_shunt


def read_current_by_current_reg(i2c):
    """
    旧算法：
    读取 Current Register，需要 Calibration 正确。
    用来和 shunt 直接算法对比。
    """
    write_reg16(i2c, INA226_ADDR, REG_CALIBRATION, CAL_VALUE)

    raw_current = read_reg16_s(i2c, INA226_ADDR, REG_CURRENT)

    current_A = raw_current * CURRENT_LSB

    if current_A < 0:
        current_A = -current_A

    current_mA = current_A * 1000.0

    return current_mA, raw_current


def read_bus_voltage(i2c):
    raw_bus = read_reg16_u(i2c, INA226_ADDR, REG_BUS_VOLTAGE)

    # INA226 Bus Voltage LSB = 1.25mV
    bus_V = raw_bus * 1.25 / 1000.0

    return bus_V, raw_bus


def scan_to_hex_text(devices):
    if devices is None or len(devices) == 0:
        return "[]"

    s = "["
    for i in range(len(devices)):
        if i > 0:
            s += ", "
        s += "0x%02X" % int(devices[i])
    s += "]"

    return s


# ==========================================================
# LCD 显示
# ==========================================================
display_mode = "lcd"
display_size = [640, 480]
rgb888p_size = [1280, 960]

pl = None


def draw_text(y, text, color=(255, 255, 255, 255), size=24):
    pl.osd_img.draw_string_advanced(
        20,
        y,
        size,
        text,
        color=color
    )


try:
    # ------------------------------
    # 1. 初始化 LCD
    # ------------------------------
    pl = PipeLine(
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        display_mode=display_mode
    )
    pl.create()

    # ------------------------------
    # 2. 初始化 IIC1
    # ------------------------------
    fpioa = FPIOA()

    # GPIO34 -> IIC1_SCL
    fpioa.set_function(
        34,
        FPIOA.IIC1_SCL,
        oe=1,
        ie=1,
        pu=1,
        st=1,
        ds=15
    )

    # GPIO35 -> IIC1_SDA
    fpioa.set_function(
        35,
        FPIOA.IIC1_SDA,
        oe=1,
        ie=1,
        pu=1,
        st=1,
        ds=15
    )

    i2c = I2C(1, scl=34, sda=35, freq=40000)

    devices = []
    ina_found = False
    init_ok = False

    smooth_current_shunt = None
    smooth_current_reg = None
    alpha = 0.25

    last_scan_ms = 0

    while True:
        now = time.ticks_ms()

        if time.ticks_diff(now, last_scan_ms) > 1000:
            last_scan_ms = now

            try:
                devices = i2c.scan()
            except Exception:
                devices = []

            if INA226_ADDR in devices:
                ina_found = True
            else:
                ina_found = False
                init_ok = False
                smooth_current_shunt = None
                smooth_current_reg = None

        pl.osd_img.clear()

        draw_text(20, "INA226 Current Check", size=30)
        draw_text(60, "IIC1: GPIO34=SCL GPIO35=SDA", color=(255, 180, 220, 255), size=22)
        draw_text(95, "Scan: " + scan_to_hex_text(devices), color=(255, 255, 255, 0), size=24)

        if not ina_found:
            draw_text(150, "INA226 NOT FOUND", color=(255, 255, 0, 0), size=32)
            draw_text(205, "Expected addr: 0x40", color=(255, 255, 255, 255), size=24)
            draw_text(245, "Check SCL/SDA/3V3/GND", color=(255, 255, 255, 255), size=22)

            pl.show_image()
            gc.collect()
            time.sleep_ms(200)
            continue

        try:
            if not init_ok:
                init_ina226(i2c)
                init_ok = True

            # 方案一：直接用分流电压计算，最可信
            current_shunt_mA, shunt_mV, raw_shunt = read_current_by_shunt(i2c)

            # 方案二：用 Current Register 计算，作为对比
            current_reg_mA, raw_current = read_current_by_current_reg(i2c)

            if smooth_current_shunt is None:
                smooth_current_shunt = current_shunt_mA
            else:
                smooth_current_shunt = smooth_current_shunt * (1.0 - alpha) + current_shunt_mA * alpha

            if smooth_current_reg is None:
                smooth_current_reg = current_reg_mA
            else:
                smooth_current_reg = smooth_current_reg * (1.0 - alpha) + current_reg_mA * alpha

            # 功率按你要求：P = I x 5V
            power_W = (smooth_current_shunt / 1000.0) * 5.0

            try:
                bus_V, raw_bus = read_bus_voltage(i2c)
            except Exception:
                bus_V = 0.0
                raw_bus = 0

            draw_text(140, "INA226 FOUND: 0x40", color=(255, 0, 255, 0), size=28)

            draw_text(
                185,
                "Current by Shunt: %.1f mA" % smooth_current_shunt,
                color=(255, 255, 255, 255),
                size=30
            )

            draw_text(
                235,
                "Power: %.3f W" % power_W,
                color=(255, 255, 255, 0),
                size=30
            )

            draw_text(
                285,
                "Current Reg: %.1f mA" % smooth_current_reg,
                color=(255, 180, 220, 255),
                size=23
            )

            draw_text(
                320,
                "Shunt: %.3f mV  raw:%d" % (shunt_mV, raw_shunt),
                color=(255, 180, 220, 255),
                size=22
            )

            draw_text(
                350,
                "Bus: %.3f V  raw:%d" % (bus_V, raw_bus),
                color=(255, 180, 220, 255),
                size=22
            )

            draw_text(
                380,
                "Rshunt: %.3f ohm" % RSHUNT,
                color=(255, 180, 220, 255),
                size=22
            )

            draw_text(
                410,
                "Cal: %d  Current_LSB: %.4fA" % (CAL_VALUE, CURRENT_LSB),
                color=(255, 180, 220, 255),
                size=20
            )

        except Exception as e:
            init_ok = False

            draw_text(150, "INA226 READ ERROR", color=(255, 255, 0, 0), size=32)
            draw_text(205, str(e), color=(255, 255, 0, 0), size=20)

        pl.show_image()
        gc.collect()
        time.sleep_ms(200)

except KeyboardInterrupt:
    print("Program stopped")

except Exception as e:
    print("Program error:", e)

finally:
    try:
        if pl:
            pl.destroy()
    except Exception:
        pass

    gc.collect()
