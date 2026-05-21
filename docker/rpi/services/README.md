# Pi host services

Lightweight systemd services that run on the Pi **host** (not inside a container).

## shutdown

GPIO button watcher. Press-and-hold (default 2 s) shorts BCM GPIO 3 to GND and
triggers a local `shutdown -h now`. GPIO 3 doubles as the Pi's halt-wake pin —
pressing it again powers the Pi back on.

### Wiring
- Button between **BCM GPIO 3 (header pin 5)** and **GND (header pin 6)**.
- No external resistor needed — uses the SoC's internal pull-up.

### Install
```bash
cd ~/vector_nav/docker/rpi/services

# Dependencies (gpiozero + lgpio pin factory)
sudo apt install -y python3-gpiozero python3-lgpio

# Install unit
sudo cp shutdown.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now shutdown.service
systemctl status shutdown.service
```

### Customize
```bash
sudo systemctl edit shutdown.service
```
Then set:
```
[Service]
Environment=SHUTDOWN_PIN=17
Environment=HOLD_TIME=3.0
```

## battery

Reads ADS1115 (I2C 0x48, AIN0) every 2 s through the same voltage divider
as the in-container `system_stats_node` (ratio 4.397). Writes a JSON
snapshot to `/run/vector/battery.json`:

```json
{"voltage": 12.41, "percent": 92.5, "ts": 1716300000.12}
```

3S LiPo curve: 12.6 V = 100 %, 11.1 V = 50 %, 9.0 V = 0 %.

### Install
```bash
sudo apt install -y python3-smbus2
sudo cp battery.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now battery.service
cat /run/vector/battery.json
```

## oled-display

128×64 SSD1306 on I2C 0x3c. Refreshes every 1 s:

```
vector-rpi
IP  192.168.8.169
WiFi <ssid> 78%
Ctrl running
Batt 92% 12.41V
```

Battery line is read from `/run/vector/battery.json` (so `battery.service`
must be running). Container status comes from `docker inspect vector-control`.

### Install
```bash
sudo apt install -y python3-luma.oled python3-pil
sudo cp oled-display.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now oled-display.service
```

## 99-lidar.rules

udev rule for the RPLidar (CH340 USB-serial). Creates `/dev/lidar`
symlink so the container always sees the LIDAR at a stable path.

```bash
sudo cp 99-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
