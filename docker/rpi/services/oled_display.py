#!/usr/bin/env python3
"""
OLED status display — SSD1306 128x64 on I2C 0x3c.

Shows hostname, IP, WiFi SSID + signal, container status, and battery
percentage/voltage (read from /run/vector/battery.json written by
battery.service).
"""

import json
import logging
import os
import socket
import subprocess
import sys
import time

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import ImageDraw, ImageFont

I2C_BUS = int(os.environ.get("I2C_BUS", "1"))
OLED_ADDR = int(os.environ.get("OLED_ADDR", "0x3c"), 0)
BATTERY_PATH = os.environ.get("BATTERY_PATH", "/run/vector/battery.json")
COUNTDOWN_PATH = os.environ.get("COUNTDOWN_PATH", "/run/vector/shutdown_countdown.json")
REFRESH_SEC = float(os.environ.get("REFRESH_SEC", "1.0"))
CONTAINER = os.environ.get("CONTAINER", "vector-control")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [oled] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


def get_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "no-net"


def get_wifi() -> tuple[str, str]:
    try:
        out = subprocess.run(
            ["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "device", "wifi"],
            capture_output=True, text=True, timeout=2,
        ).stdout
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[0] == "yes":
                return parts[1], f"{parts[2]}%"
    except Exception:
        pass
    return "—", ""


def get_container_state() -> str:
    try:
        out = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Status}}", CONTAINER],
            capture_output=True, text=True, timeout=2,
        )
        return out.stdout.strip() or "?"
    except Exception:
        return "?"


def get_countdown() -> int | None:
    try:
        with open(COUNTDOWN_PATH) as f:
            d = json.load(f)
        if time.time() - d.get("ts", 0) > 3:
            return None
        return int(d.get("remaining", 0))
    except FileNotFoundError:
        return None
    except Exception:
        return None


def get_battery() -> tuple[str, str]:
    try:
        with open(BATTERY_PATH) as f:
            d = json.load(f)
        if time.time() - d.get("ts", 0) > 10:
            return "stale", ""
        return f"{d['percent']:.0f}%", f"{d['voltage']:.2f}V"
    except Exception:
        return "—", ""


def main():
    serial = i2c(port=I2C_BUS, address=OLED_ADDR)
    device = ssd1306(serial, width=128, height=64)
    font = ImageFont.load_default()
    try:
        big_font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 40)
    except Exception:
        big_font = font

    hostname = socket.gethostname()
    log.info("OLED started on i2c-%d @ 0x%02x", I2C_BUS, OLED_ADDR)

    from luma.core.render import canvas

    while True:
        countdown = get_countdown()
        if countdown is not None:
            with canvas(device) as draw:
                draw.text((0, 0), "SHUTDOWN IN", font=font, fill=255)
                txt = str(countdown)
                bbox = draw.textbbox((0, 0), txt, font=big_font)
                w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
                draw.text(((128 - w) // 2, 16), txt, font=big_font, fill=255)
            time.sleep(0.25)
            continue

        ip = get_ip()
        ssid, sig = get_wifi()
        ctn = get_container_state()
        batt_pct, batt_v = get_battery()
        with canvas(device) as draw:
            draw.text((0, 0), hostname, font=font, fill=255)
            draw.text((0, 12), f"IP  {ip}", font=font, fill=255)
            draw.text((0, 24), f"WiFi {ssid} {sig}", font=font, fill=255)
            draw.text((0, 36), f"Ctrl {ctn}", font=font, fill=255)
            draw.text((0, 48), f"Batt {batt_pct} {batt_v}", font=font, fill=255)
        time.sleep(REFRESH_SEC)


if __name__ == "__main__":
    main()
