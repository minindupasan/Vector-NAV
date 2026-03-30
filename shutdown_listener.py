#!/usr/bin/env python3
"""
Shutdown Listener Service — Jetson side
========================================
Listens on TCP port 9877 for a "SHUTDOWN" command from the Raspberry Pi.
On receipt it acknowledges and issues a local shutdown.

Copy this file to the Jetson and run it as a systemd service (see the
comments at the bottom for the unit file).

Security: only connections from ALLOWED_IPS are acted upon.  Set this to
the Pi's static IP or leave empty to allow any source (trusted LAN only).
"""

import logging
import socket
import subprocess
import sys

# ── Configuration ────────────────────────────────────────────────────────────
LISTEN_HOST  = "0.0.0.0"   # listen on all interfaces
LISTEN_PORT  = 9877         # must match JETSON_PORT in shutdown_button.py
ALLOWED_IPS  = set()        # e.g. {"192.168.1.50"} — empty means accept all
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [shutdown-listener] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


def handle_connection(conn: socket.socket, addr: tuple):
    client_ip = addr[0]

    if ALLOWED_IPS and client_ip not in ALLOWED_IPS:
        log.warning("Rejected connection from %s (not in ALLOWED_IPS).", client_ip)
        conn.close()
        return

    try:
        data = conn.recv(32).strip().decode("ascii", errors="replace")
        log.info("Received %r from %s.", data, client_ip)

        if data == "SHUTDOWN":
            conn.sendall(b"OK\n")
            conn.close()
            log.info("Shutdown command accepted — shutting down.")
            subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)
        else:
            conn.sendall(b"ERR unknown command\n")
            conn.close()
            log.warning("Unknown command %r — ignored.", data)
    except OSError as e:
        log.error("Socket error with %s: %s", client_ip, e)
    finally:
        try:
            conn.close()
        except OSError:
            pass


def main():
    log.info("Shutdown listener starting on %s:%d.", LISTEN_HOST, LISTEN_PORT)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((LISTEN_HOST, LISTEN_PORT))
        srv.listen(1)
        log.info("Listening …")

        while True:
            try:
                conn, addr = srv.accept()
                log.info("Connection from %s:%d.", *addr)
                handle_connection(conn, addr)
            except KeyboardInterrupt:
                log.info("Interrupted — exiting.")
                break
            except OSError as e:
                log.error("Accept error: %s", e)


if __name__ == "__main__":
    main()


# ── Jetson systemd unit (save as /etc/systemd/system/shutdown-listener.service)
# ──────────────────────────────────────────────────────────────────────────────
# [Unit]
# Description=Shutdown Listener (triggered by Raspberry Pi button)
# After=network.target
#
# [Service]
# Type=simple
# ExecStart=/usr/bin/python3 /home/<JETSON_USER>/shutdown_listener.py
# Restart=always
# RestartSec=3
# StandardOutput=journal
# StandardError=journal
#
# [Install]
# WantedBy=multi-user.target
# ──────────────────────────────────────────────────────────────────────────────
# Enable with:
#   sudo systemctl daemon-reload
#   sudo systemctl enable --now shutdown-listener.service
