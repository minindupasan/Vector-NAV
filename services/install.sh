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

install_service vector-assistant.service
install_service vector-web.service

# Allow admin to run Jetson power/clock tools without a password prompt
# (needed by run_assistant.sh which runs as the admin user under systemd)
SUDOERS_FILE=/etc/sudoers.d/vector-nav-jetson
if [ ! -f "$SUDOERS_FILE" ]; then
    echo "Creating sudoers entry for nvpmodel / jetson_clocks..."
    echo 'admin ALL=(ALL) NOPASSWD: /usr/sbin/nvpmodel, /usr/bin/jetson_clocks' \
        | sudo tee "$SUDOERS_FILE" > /dev/null
    sudo chmod 440 "$SUDOERS_FILE"
fi

sudo systemctl daemon-reload

# Enable linger so user services (like PulseAudio) run without an active session
echo "Enabling linger for admin user..."
sudo loginctl enable-linger admin

echo ""
echo "Services installed. To enable on boot:"
echo "  sudo systemctl enable vector-assistant vector-web"
echo ""
echo "To start now:"
echo "  sudo systemctl start vector-assistant"
echo "  sudo systemctl start vector-web"
echo ""
echo "To check status:"
echo "  sudo systemctl status vector-assistant"
echo "  sudo systemctl status vector-web"
echo ""
echo "To follow logs:"
echo "  journalctl -u vector-assistant -f"
echo "  journalctl -u vector-web -f"
