# K230 颜色追踪 + UART 坐标输出 (基于已验证的 vision 代码)
# 接线: K230 IO3(TX) → STM32 PB11(USART3_RX)
#       K230 GND     → STM32 GND

import time, os, gc, sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ============ UART1 (TX=IO3 默认, 115200) ============
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)   # IO3 → UART1 TX

uart = UART(UART.UART1, baudrate=115200,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE)

# ============ 颜色阈值 (和你原版一致) ============
COLOR_THRESHOLDS = [
    ("RED",    (0,   80,  20,  80,  -20,  80),  (255, 0,   0)),
]

MIN_AREA    = 2000    # 大一点, 过滤小色块干扰
H_MIRROR    = True
V_FLIP      = True
SEND_MS     = 50       # 发送间隔 ms

DISP_WIDTH  = 800
DISP_HEIGHT = 480
SENSOR_W    = 640
SENSOR_H    = 480

print("=== K230 Color Tracking + UART ===")

# ---- 相机 ----
print("[1/3] Init Sensor...")
sensor = Sensor(width=SENSOR_W, height=SENSOR_H)
sensor.reset()
sensor.set_framesize(width=SENSOR_W, height=SENSOR_H)
sensor.set_pixformat(Sensor.RGB565)
try:
    sensor.set_hmirror(H_MIRROR)
    sensor.set_vflip(V_FLIP)
except:
    pass

# ---- 显示 ----
print("[2/3] Init Display...")
Display.init(Display.ST7701, width=DISP_WIDTH, height=DISP_HEIGHT, to_ide=True)
MediaManager.init()
sensor.run()

print("[3/3] Running...  UART1 TX=IO3, 115200 baud\n")

fps_clock = time.clock()
last_send = time.ticks_ms()

try:
    while True:
        fps_clock.tick()
        os.exitpoint()

        img = sensor.snapshot()
        if img is None:
            continue

        # 找所有颜色中的最大色块
        best_blob = None
        best_area = 0
        best_name = ""
        best_color = (255, 255, 255)

        for color_name, threshold, display_color in COLOR_THRESHOLDS:
            blobs = img.find_blobs(
                [threshold],
                pixels_threshold=MIN_AREA,
                area_threshold=MIN_AREA,
                merge=True
            )
            for blob in blobs:
                if blob.area() > best_area:
                    best_area = blob.area()
                    best_blob = blob
                    best_name = color_name
                    best_color = display_color

        # 只显示最大那个色块
        if best_blob:
            img.draw_rectangle([v for v in best_blob.rect()],
                               color=best_color, thickness=3)
            img.draw_cross(best_blob.cx(), best_blob.cy(),
                           color=best_color, size=10, thickness=2)
            label = "%s(%d)" % (best_name, best_blob.area())
            img.draw_string_advanced(
                best_blob.x(), max(0, best_blob.y() - 25),
                24, label, color=best_color
            )

        # UART 发送 + 串口打印
        now = time.ticks_ms()
        if best_blob and time.ticks_diff(now, last_send) >= SEND_MS:
            cx = best_blob.cx()
            cy = best_blob.cy()
            data = f"{cx},{cy}\n"
            uart.write(data)
            print(f"CX={cx}, CY={cy}, Area={best_area}, Color={best_name}")
            last_send = now

        # FPS
        fps_text = "FPS: %.1f  Track: %s" % (fps_clock.fps(), best_name)
        img.draw_string_advanced(4, 4, 28, fps_text,
                                 color=(255, 0, 255, 0))

        Display.show_image(img)

        if not best_blob:
            gc.collect()

except KeyboardInterrupt:
    print("\n  Stopped by user.")
except Exception as e:
    print("\n  Error: %s - %s" % (type(e).__name__, e))
finally:
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    time.sleep_ms(100)
    gc.collect()
    print("  Done.")
