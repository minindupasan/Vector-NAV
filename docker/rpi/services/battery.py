#!/usr/bin/env python3
"""
Battery monitor — reads ADS1115 (I2C 0x48, AIN0) and writes a JSON
snapshot of pack voltage / percentage to /run/vector/battery.json
every POLL_SEC seconds.

3S LiPo voltage divider scales pack down to ADC range; DIVIDER_RATIO
matches the existing system_stats_node in the control container.
"""

import json
import logging
import os
import signal
import socket
import subprocess
import sys
import time

from smbus2 import SMBus, i2c_msg

I2C_BUS = int(os.environ.get("I2C_BUS", "1"))
ADS_ADDR = 0x48
DIVIDER_RATIO = 4.3178
POLL_SEC = float(os.environ.get("POLL_SEC", "2.0"))
OUT_PATH = os.environ.get("OUT_PATH", "/run/vector/battery.json")
SHUTDOWN_VOLTAGE = float(os.environ.get("SHUTDOWN_VOLTAGE", "10.0"))
JETSON_HOST = os.environ.get("JETSON_HOST", "192.168.10.1")
JETSON_PORT = int(os.environ.get("JETSON_PORT", "9877"))

# 3S LiPo discharge curve (resting voltage)
_TABLE = [
    (12.6, 100), (12.0, 90), (11.6, 75), (11.1, 50),
    (10.5, 25), (9.9, 10), (9.0, 0),
]

# ADS1115 config: AIN0 single-ended, FS=±4.096V, single-shot, 128 SPS
ADS_CONFIG = 0xC383
REG_CONVERSION = 0x00
REG_CONFIG = 0x01

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [battery] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


def read_voltage(bus: SMBus) -> float:
    hi, lo = (ADS_CONFIG >> 8) & 0xFF, ADS_CONFIG & 0xFF
    bus.write_i2c_block_data(ADS_ADDR, REG_CONFIG, [hi, lo])
    time.sleep(0.012)
    data = bus.read_i2c_block_data(ADS_ADDR, REG_CONVERSION, 2)
    raw = (data[0] << 8) | data[1]
    if raw & 0x8000:
        raw -= 1 << 16
    adc_v = (raw * 4.096) / 32768.0
    return adc_v * DIVIDER_RATIO


def voltage_to_percent(v: float) -> float:
    if v >= _TABLE[0][0]:
        return 100.0
    if v <= _TABLE[-1][0]:
        return 0.0
    for (v_hi, p_hi), (v_lo, p_lo) in zip(_TABLE, _TABLE[1:]):
        if v_lo <= v <= v_hi:
            return p_lo + (p_hi - p_lo) * (v - v_lo) / (v_hi - v_lo)
    return 0.0


def signal_jetson() -> None:
    try:
        with socket.create_connection((JETSON_HOST, JETSON_PORT), timeout=2.0) as s:
            s.sendall(b"SHUTDOWN\n")
            ack = s.recv(16).decode("ascii", errors="replace").strip()
            log.info("Jetson ack: %r", ack)
    except Exception as e:
        log.warning("Could not signal Jetson at %s:%d — %s", JETSON_HOST, JETSON_PORT, e)


def write_atomic(payload: dict) -> None:
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    tmp = OUT_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, OUT_PATH)


def main():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    log.info("Reading ADS1115 0x%02X on i2c-%d every %.1fs → %s",
             ADS_ADDR, I2C_BUS, POLL_SEC, OUT_PATH)

    ema_v = None
    alpha = 0.3
    with SMBus(I2C_BUS) as bus:
        while True:
            try:
                v = read_voltage(bus)
                ema_v = v if ema_v is None else (alpha * v + (1 - alpha) * ema_v)
                payload = {
                    "voltage": round(ema_v, 3),
                    "percent": round(voltage_to_percent(ema_v), 1),
                    "ts": time.time(),
                }
                write_atomic(payload)
                if v <= SHUTDOWN_VOLTAGE and ema_v <= SHUTDOWN_VOLTAGE:
                    log.critical(
                        "Battery critical: raw=%.2fV EMA=%.2fV ≤ %.1fV — shutting down NOW!",
                        v, ema_v, SHUTDOWN_VOLTAGE,
                    )
                    signal_jetson()
                    subprocess.run(["/sbin/shutdown", "-h", "now"], check=False)
                    sys.exit(0)
            except Exception as e:
                log.warning("Read failed: %s", e)
            time.sleep(POLL_SEC)


if __name__ == "__main__":
    main()
