#!/usr/bin/env python3
"""USB 相机测试脚本：打开相机、实时显示画面、统计帧率。

用法示例：
    python3 test_usb_camera.py                     # 默认 /dev/video0, 1360x480, MJPG
    python3 test_usb_camera.py -d 0 -W 2720 -H 960 # 指定设备和分辨率
    python3 test_usb_camera.py --fourcc YUYV       # 换成 YUYV 格式

快捷键：
    q / ESC   退出
    s         保存当前帧为 jpg
"""
import argparse
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="USB 相机测试")
    p.add_argument("-d", "--device", type=int, default=0, help="设备号 (/dev/videoN)，默认 0")
    p.add_argument("-W", "--width", type=int, default=1360, help="画面宽度，默认 1360")
    p.add_argument("-H", "--height", type=int, default=480, help="画面高度，默认 480")
    p.add_argument("--fps", type=int, default=30, help="请求帧率，默认 30")
    p.add_argument("--fourcc", default="MJPG", help="像素格式 fourcc，默认 MJPG（可选 YUYV）")
    return p.parse_args()


def main():
    args = parse_args()

    # 用 V4L2 后端打开，避免 GStreamer 默认协商失败
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[错误] 无法打开 /dev/video{args.device}")
        return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    # 打印实际生效的参数
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[信息] 设备=/dev/video{args.device} 分辨率={real_w}x{real_h} 格式={args.fourcc} 请求FPS={real_fps}")
    print("[提示] 按 q 或 ESC 退出，按 s 保存当前帧")

    win = f"USB Camera /dev/video{args.device}"
    frames = 0
    fps = 0.0
    t0 = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[错误] 读取帧失败")
            break

        # 每秒计算一次实测帧率
        frames += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps = frames / elapsed
            frames = 0
            t0 = time.time()

        cv2.putText(frame, f"{real_w}x{real_h}  FPS:{fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(win, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # q 或 ESC
            break
        if key == ord("s"):
            fname = f"capture_{int(time.time())}.jpg"
            cv2.imwrite(fname, frame)
            print(f"[保存] {fname}")

    cap.release()
    cv2.destroyAllWindows()
    print("[信息] 已退出")


if __name__ == "__main__":
    main()
