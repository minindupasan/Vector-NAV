#!/usr/bin/env python3
"""
Shutdown Button — Raspberry Pi host service.

Watches a GPIO pin (active-low, internal pull-up). Hold for HOLD_TIME
seconds to arm the shutdown sequence:

  1. Write /run/vector/shutdown_countdown.json each second
     (so oled-display.service can render a countdown screen).
  2. At T=0, send "SHUTDOWN\\n" to JETSON_HOST:JETSON_PORT so the
     Jetson powers down (shutdown-listener.service on the other side).
  3. Then issue `shutdown -h now` locally.

Env overrides:
  SHUTDOWN_PIN   BCM pin (default 3)
  HOLD_TIME      seconds to arm (default 2.0)
  COUNTDOWN_SEC  countdown length (default 5)
  JETSON_HOST    Jetson IP/hostname (default 192.168.8.143)
  JETSON_PORT    TCP port of shutdown-listener (default 9877)
  COUNTDOWN_PATH JSON path (default /run/vector/shutdown_countdown.json)
"""

import json
import logging
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from signal import pause

from gpiozero import Button, Device
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

PIN = int(os.environ.get("SHUTDOWN_PIN", "3"))
HOLD_TIME = float(os.environ.get("HOLD_TIME", "2.0"))
COUNTDOWN_SEC = int(os.environ.get("COUNTDOWN_SEC", "5"))
JETSON_HOST = os.environ.get("JETSON_HOST", "192.168.10.1")
JETSON_PORT = int(os.environ.get("JETSON_PORT", "9877"))
COUNTDOWN_PATH = os.environ.get("COUNTDOWN_PATH", "/run/vector/shutdown_countdown.json")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [shutdown-button] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


def write_countdown(remaining: int) -> None:
    os.makedirs(os.path.dirname(COUNTDOWN_PATH), exist_ok=True)
    tmp = COUNTDOWN_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"remaining": remaining, "ts": time.time()}, f)
    os.replace(tmp, COUNTDOWN_PATH)


def clear_countdown() -> None:
    try:
        os.remove(COUNTDOWN_PATH)
    except FileNotFoundError:
        pass


def signal_jetson() -> None:
    try:
        with socket.create_connection((JETSON_HOST, JETSON_PORT), timeout=2.0) as s:
            s.sendall(b"SHUTDOWN\n")
            ack = s.recv(16).decode("ascii", errors="replace").strip()
            log.info("Jetson ack: %r", ack)
    except Exception as e:
        log.warning("Could not signal Jetson at %s:%d — %s",
                    JETSON_HOST, JETSON_PORT, e)


counting = threading.Event()
abort = threading.Event()


def on_held():
    if counting.is_set():
        return
    counting.set()
    abort.clear()
    log.warning("Button armed — %ds countdown begins (press again to abort).",
                COUNTDOWN_SEC)
    try:
        for remaining in range(COUNTDOWN_SEC, 0, -1):
            write_countdown(remaining)
            log.info("Shutdown in %d…", remaining)
            if abort.wait(timeout=1.0):
                log.warning("Shutdown aborted by button press.")
                clear_countdown()
                return
        write_countdown(0)
        log.warning("Signalling Jetson and powering off.")
        signal_jetson()
        subprocess.run(["/sbin/shutdown", "-h", "now"], check=False)
    finally:
        counting.clear()


def on_pressed():
    if counting.is_set():
        abort.set()


def main():
    clear_countdown()
    button = Button(PIN, pull_up=True, hold_time=HOLD_TIME, bounce_time=0.05)
    button.when_held = on_held
    button.when_pressed = on_pressed
    log.info("Watching GPIO %d (hold %.1fs → %ds countdown → poweroff).",
             PIN, HOLD_TIME, COUNTDOWN_SEC)

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    pause()


if __name__ == "__main__":
    main()
