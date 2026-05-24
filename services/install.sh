#!/bin/bash
# Vector Nav — service manager
#
# Usage:
#   bash services/install.sh                  # install/update all services
#   bash services/install.sh start <svc>      # start   (e.g. vector-web)
#   bash services/install.sh stop <svc>       # stop
#   bash services/install.sh restart <svc>    # restart
#   bash services/install.sh enable <svc>     # enable on boot
#   bash services/install.sh disable <svc>    # disable on boot
#   bash services/install.sh status <svc>     # show status
#   bash services/install.sh logs <svc>       # follow journal
#   bash services/install.sh start all        # start all services
#   bash services/install.sh restart all      # restart all services
#
# Omit <svc> suffix to default to all services listed in ALL_SERVICES.

set -e
SERVICES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ALL_SERVICES=(vector-llm vector-assistant vector-web vector-navigation vector-status vector-camera)

# ── Helpers ──────────────────────────────────────────────────────────────────

_svc_name() {
    # Normalise: add .service suffix if missing
    local s="$1"
    [[ "$s" == *.service ]] || s="${s}.service"
    echo "$s"
}

run_for() {
    # run_for <action_fn> [svc|all]
    local fn="$1"; local target="${2:-all}"
    if [[ "$target" == "all" ]]; then
        for s in "${ALL_SERVICES[@]}"; do $fn "$s"; done
    else
        $fn "$target"
    fi
}

do_install() {
    local name
    name="$(_svc_name "$1")"
    if [[ ! -f "$SERVICES_DIR/$name" ]]; then
        echo "  [skip] $name — file not found in services/"
        return
    fi
    echo "  Installing $name..."
    sudo cp "$SERVICES_DIR/$name" /etc/systemd/system/
    sudo chmod 644 /etc/systemd/system/"$name"
}

do_start()   { echo "  Starting $1...";   sudo systemctl start   "$(_svc_name "$1")"; }
do_stop()    { echo "  Stopping $1...";   sudo systemctl stop    "$(_svc_name "$1")"; }
do_restart() { echo "  Restarting $1..."; sudo systemctl restart "$(_svc_name "$1")"; }
do_enable()  { echo "  Enabling $1...";   sudo systemctl enable  "$(_svc_name "$1")"; }
do_disable() { echo "  Disabling $1...";  sudo systemctl disable "$(_svc_name "$1")"; }
do_status()  { sudo systemctl status --no-pager "$(_svc_name "$1")"; }
do_logs()    { journalctl -u "$(_svc_name "$1")" -f; }

# ── Dispatch ─────────────────────────────────────────────────────────────────

ACTION="${1:-install}"
TARGET="${2:-all}"

case "$ACTION" in
    install|update)
        echo "Installing / updating Vector Nav services..."
        for s in "${ALL_SERVICES[@]}"; do do_install "$s"; done

        # Sudoers rule (idempotent — only writes if file is missing)
        SUDOERS_FILE=/etc/sudoers.d/vector-nav-jetson
        if [ ! -f "$SUDOERS_FILE" ]; then
            echo "  Creating sudoers entry..."
            echo 'admin ALL=(ALL) NOPASSWD: /usr/sbin/nvpmodel, /usr/bin/jetson_clocks, /usr/bin/docker, /usr/bin/systemctl start vector-*, /usr/bin/systemctl stop vector-*, /usr/bin/systemctl restart vector-*, /usr/bin/systemctl enable vector-*, /usr/bin/systemctl disable vector-*, /usr/bin/systemctl status vector-*' \
                | sudo tee "$SUDOERS_FILE" > /dev/null
            sudo chmod 440 "$SUDOERS_FILE"
        fi

        # Web panel sudoers (service restart/stop + power — always kept in sync)
        PANEL_SUDOERS=/etc/sudoers.d/vector-web-panel
        echo "  Installing web panel sudoers..."
        sudo cp "$SERVICES_DIR/vector-web-panel" "$PANEL_SUDOERS"
        sudo chmod 440 "$PANEL_SUDOERS"
        sudo visudo -c -f "$PANEL_SUDOERS" || { echo "  [ERROR] sudoers syntax invalid — removing"; sudo rm "$PANEL_SUDOERS"; exit 1; }

        sudo systemctl daemon-reload
        sudo loginctl enable-linger admin

        echo ""
        echo "Done. Quick reference:"
        echo "  bash services/install.sh start   all"
        echo "  bash services/install.sh restart vector-web"
        echo "  bash services/install.sh enable  all"
        echo "  bash services/install.sh logs    vector-navigation"
        ;;
    start)   run_for do_start   "$TARGET" ;;
    stop)    run_for do_stop    "$TARGET" ;;
    restart) run_for do_restart "$TARGET" ;;
    enable)  run_for do_enable  "$TARGET" ;;
    disable) run_for do_disable "$TARGET" ;;
    status)  run_for do_status  "$TARGET" ;;
    logs)    do_logs "$TARGET" ;;
    *)
        echo "Unknown action: $ACTION"
        echo "Usage: bash services/install.sh [install|start|stop|restart|enable|disable|status|logs] [service|all]"
        exit 1
        ;;
esac
