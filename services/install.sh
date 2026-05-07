#!/bin/bash
# Install Vector Nav systemd services.
# Run once as a user with sudo rights:  bash services/install.sh

set -e
SERVICES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install_service() {
    local name="$1"
    echo "Installing $name..."
    sudo cp "$SERVICES_DIR/$name" /etc/systemd/system/
    sudo chmod 644 /etc/systemd/system/"$name"
}

install_service vector-llm.service
install_service vector-web.service

# Allow admin to run Jetson power/clock tools without a password prompt
# (needed by run_llm_headless.sh which runs as the admin user under systemd)
SUDOERS_FILE=/etc/sudoers.d/vector-nav-jetson
if [ ! -f "$SUDOERS_FILE" ]; then
    echo "Creating sudoers entry for nvpmodel / jetson_clocks..."
    echo 'admin ALL=(ALL) NOPASSWD: /usr/sbin/nvpmodel, /usr/bin/jetson_clocks' \
        | sudo tee "$SUDOERS_FILE" > /dev/null
    sudo chmod 440 "$SUDOERS_FILE"
fi

sudo systemctl daemon-reload

echo ""
echo "Services installed. To enable on boot:"
echo "  sudo systemctl enable vector-llm vector-web"
echo ""
echo "To start now:"
echo "  sudo systemctl start vector-llm"
echo "  sudo systemctl start vector-web"
echo ""
echo "To check status:"
echo "  sudo systemctl status vector-llm"
echo "  sudo systemctl status vector-web"
echo ""
echo "To follow logs:"
echo "  journalctl -u vector-llm -f"
echo "  journalctl -u vector-web -f"
