# IMX519 Dual Camera V-Link Setup for Jetson Orin Nano Super

Device tree overlay for running dual IMX519 cameras with Videtronic V-Link (MAX96717/MAX96714) serializer/deserializer on Jetson Orin Nano Super.

## Hardware Requirements

- Jetson Orin Nano Super Developer Kit
- 2x Arducam IMX519 cameras
- 2x Videtronic V-Link kits (serializer + deserializer per camera)
- JetPack 6.x (L4T 36.4.x)

## Camera Port Configuration

| Camera | CSI Port | I2C Bus | Serial Interface | lane_polarity |
|--------|----------|---------|------------------|---------------|
| CAM0   | Port 1   | i2c@0   | serial_b         | 6             |
| CAM1   | Port 2   | i2c@1   | serial_c         | (none)        |

**Important:** CAM1 (serial_c) must NOT have `lane_polarity` set. This is a known requirement for CSI Port 2 on Jetson Orin Nano.

## Installation

### Prerequisites

#### 1. Arducam IMX519 Driver Package

The Arducam IMX519 driver must be installed. Follow the official Arducam instructions:

```bash
# Check if already installed
dpkg -l | grep arducam

# If not installed, download and install from Arducam:
# https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/Quick-Start-Guide/
# 
# For JetPack 6.x (L4T 36.x):
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/arducam-nvidia-l4t-kernel_36.4-2024.12.18-1_arm64.deb
sudo dpkg -i arducam-nvidia-l4t-kernel_36.4-2024.12.18-1_arm64.deb

# Reboot after installation
sudo reboot
```

#### 2. Videtronic V-Link Drivers

The V-Link serializer (MAX96717) and deserializer (MAX96714) drivers are included in this repository.

```bash
# Build and install the serializer driver
cd v-link-ser
make
sudo make install

# Build and install the deserializer driver
cd ../v-link-deser
make
sudo make install

# Load the modules (or reboot)
sudo modprobe v-link-ser
sudo modprobe v-link-deser

# Verify drivers are loaded
lsmod | grep v-link
# Should show: v-link_ser and v-link_deser
```

To make the drivers load automatically on boot:
```bash
echo "v-link-ser" | sudo tee -a /etc/modules-load.d/v-link.conf
echo "v-link-deser" | sudo tee -a /etc/modules-load.d/v-link.conf
```

### Install the Overlay

1. Copy the compiled overlay to /boot:
   ```bash
   sudo cp tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo /boot/
   ```

2. Edit `/boot/extlinux/extlinux.conf` and add/modify the OVERLAYS line:
   ```
   OVERLAYS /boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo
   ```

   Example full entry:
   ```
   LABEL primary
         MENU LABEL primary kernel
         LINUX /boot/Image
         INITRD /boot/initrd
         FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
         OVERLAYS /boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo
         APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait rootfstype=ext4 ...
   ```

3. Reboot:
   ```bash
   sudo reboot
   ```

### Verify Installation

After reboot, check that both cameras are detected:

```bash
# Check video devices
ls /dev/video*
# Should show: /dev/video0 /dev/video1

# Check kernel logs
sudo dmesg | grep imx519
# Should show both cameras bound:
# imx519 9-001a: tegracam sensor driver:imx519_v2.0.6
# tegra-camrtc-capture-vi tegra-capture-vi: subdev imx519 9-001a bound
# imx519 10-001a: tegracam sensor driver:imx519_v2.0.6
# tegra-camrtc-capture-vi tegra-capture-vi: subdev imx519 10-001a bound
```

## Testing Cameras

Test CAM0:
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

Test CAM1:
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! xvimagesink
```

## Available Sensor Modes

| Mode | Resolution  | FPS | Use Case           |
|------|-------------|-----|--------------------|
| 0    | 4656x3496   | 9   | Full resolution    |
| 1    | 3840x2160   | 17  | 4K video           |
| 2    | 1920x1080   | 60  | 1080p high FPS     |
| 3    | 1280x720    | 120 | 720p highest FPS   |

## Rebuilding the Overlay

If you need to modify and rebuild the overlay:

```bash
dtc -I dts -O dtb -@ tegra234-p3767-camera-p3768-imx519-vlink-dual.dts \
    -o tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo
```

## Troubleshooting

### CAM1 stops after ~4 seconds

This is caused by incorrect `lane_polarity` setting for serial_c. Ensure CAM1 modes do NOT have `lane_polarity` set.

### "Timeout!! Skipping requests on sensor GUID 1"

Same issue as above - check that lane_polarity is removed from all CAM1 modes.

### "Failed to create CaptureSession"

Restart nvargus-daemon:
```bash
sudo systemctl restart nvargus-daemon
```

### Only one camera detected

Check that both V-Link serializers and deserializers are probed:
```bash
sudo dmesg | grep v-link
# Should show 4 entries: v-link-ser 9-0042, v-link-ser 10-0042,
#                        v-link-deser 9-004c, v-link-deser 10-004c
```

## Files

```
imx519-vlink-dual/
├── README.md                                              # This file
├── tegra234-p3767-camera-p3768-imx519-vlink-dual.dts      # Device tree source
├── tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo     # Compiled overlay
├── v-link-ser/                                            # Serializer driver
│   ├── Makefile
│   └── v-link-ser.c
└── v-link-deser/                                          # Deserializer driver
    ├── Makefile
    └── v-link-deser.c
```

## Tested Configuration

- Jetson Orin Nano Super Developer Kit
- JetPack 6 (L4T 36.4.4)
- Kernel: 5.15.148-tegra
- Arducam IMX519 cameras
- Videtronic V-Link (MAX96717/MAX96714)

## License

This overlay is based on Videtronic V-Link drivers and Arducam IMX519 driver configurations.
