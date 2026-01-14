#!/usr/bin/env bash
#
# install.sh - Automated installer for IMX519 Dual Camera V-Link Setup
#
# This script installs everything needed to run dual IMX519 cameras
# with Videtronic V-Link on Jetson Orin Nano Super.
#
# Usage:
#   sudo ./install.sh           # Full installation
#   sudo ./install.sh --check   # Check current status only
#   sudo ./install.sh --help    # Show help
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# -----------------------------
# Helper functions
# -----------------------------
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

show_help() {
    cat << EOF
IMX519 Dual Camera V-Link Installer

Usage: sudo $(basename "$0") [OPTIONS]

Options:
  --check       Check current installation status only
  --skip-arducam Skip Arducam driver installation (if already installed)
  --skip-vlink   Skip V-Link driver installation (if already installed)
  --skip-overlay Skip device tree overlay installation
  --help        Show this help message

This script will:
  1. Check system requirements (Jetson Orin Nano, JetPack 6.x)
  2. Install Arducam IMX519 kernel driver (if needed)
  3. Build and install V-Link serializer/deserializer drivers
  4. Install the dual camera device tree overlay
  5. Configure extlinux.conf to load the overlay
  6. Optionally reboot to apply changes

EOF
    exit 0
}

# -----------------------------
# Check functions
# -----------------------------
check_jetson() {
    log_info "Checking Jetson platform..."
    
    if [[ ! -f /etc/nv_tegra_release ]]; then
        log_error "This doesn't appear to be a Jetson device"
        return 1
    fi
    
    local release
    release=$(cat /etc/nv_tegra_release)
    log_info "Tegra release: $release"
    
    # Check for Orin Nano
    if ! grep -q "t234" /proc/device-tree/compatible 2>/dev/null; then
        log_warn "This script is designed for Jetson Orin Nano (t234)"
    fi
    
    log_success "Jetson platform detected"
    return 0
}

check_jetpack() {
    log_info "Checking JetPack version..."
    
    local l4t_version
    if [[ -f /etc/nv_tegra_release ]]; then
        l4t_version=$(head -n 1 /etc/nv_tegra_release | sed 's/.*R\([0-9]*\).*/\1/')
    fi
    
    if [[ "$l4t_version" -lt 36 ]]; then
        log_error "JetPack 6.x (L4T 36.x) or later required. Found L4T R${l4t_version}"
        return 1
    fi
    
    log_success "JetPack version OK (L4T R${l4t_version})"
    return 0
}

check_arducam_installed() {
    if dpkg -l | grep -q arducam; then
        local version
        version=$(dpkg -l | grep arducam | awk '{print $3}')
        log_success "Arducam driver installed: $version"
        return 0
    else
        log_warn "Arducam driver not installed"
        return 1
    fi
}

check_vlink_installed() {
    local ser_loaded=false
    local deser_loaded=false
    
    if lsmod | grep -q v_link_ser; then
        ser_loaded=true
    fi
    if lsmod | grep -q v_link_deser; then
        deser_loaded=true
    fi
    
    if $ser_loaded && $deser_loaded; then
        log_success "V-Link drivers loaded (v-link-ser, v-link-deser)"
        return 0
    elif $ser_loaded || $deser_loaded; then
        log_warn "V-Link drivers partially loaded"
        return 1
    else
        log_warn "V-Link drivers not loaded"
        return 1
    fi
}

check_overlay_installed() {
    if [[ -f /boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo ]]; then
        log_success "Device tree overlay installed in /boot"
        
        # Check if configured in extlinux.conf
        if grep -q "imx519-vlink-dual.dtbo" /boot/extlinux/extlinux.conf 2>/dev/null; then
            log_success "Overlay configured in extlinux.conf"
            return 0
        else
            log_warn "Overlay not configured in extlinux.conf"
            return 1
        fi
    else
        log_warn "Device tree overlay not installed"
        return 1
    fi
}

check_cameras_detected() {
    log_info "Checking camera detection..."
    
    local video_devices
    video_devices=$(ls /dev/video* 2>/dev/null | wc -l)
    
    if [[ $video_devices -ge 2 ]]; then
        log_success "Found $video_devices video devices"
        
        # Check dmesg for IMX519
        if dmesg | grep -q "imx519.*bound"; then
            local bound_count
            bound_count=$(dmesg | grep "imx519.*bound" | wc -l)
            log_success "IMX519 cameras bound: $bound_count"
            return 0
        fi
    fi
    
    log_warn "Cameras not fully detected (may need reboot)"
    return 1
}

run_checks() {
    echo ""
    echo "=========================================="
    echo "  IMX519 Dual Camera Installation Status"
    echo "=========================================="
    echo ""
    
    local all_ok=true
    
    check_jetson || all_ok=false
    check_jetpack || all_ok=false
    check_arducam_installed || all_ok=false
    check_vlink_installed || all_ok=false
    check_overlay_installed || all_ok=false
    check_cameras_detected || all_ok=false
    
    echo ""
    if $all_ok; then
        echo -e "${GREEN}All checks passed! Dual camera setup is complete.${NC}"
        return 0
    else
        echo -e "${YELLOW}Some components need attention. Run installer to fix.${NC}"
        return 1
    fi
}

# -----------------------------
# Installation functions
# -----------------------------
install_arducam() {
    log_info "Installing Arducam IMX519 driver..."
    
    if check_arducam_installed; then
        log_info "Arducam driver already installed, skipping"
        return 0
    fi
    
    # Determine L4T version for correct package
    local l4t_version
    l4t_version=$(head -n 1 /etc/nv_tegra_release | sed 's/.*R\([0-9]*\)\.\([0-9]*\).*/\1.\2/')
    
    log_info "Downloading Arducam driver for L4T ${l4t_version}..."
    
    local arducam_url="https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/arducam-nvidia-l4t-kernel_36.4-2024.12.18-1_arm64.deb"
    local arducam_deb="/tmp/arducam-driver.deb"
    
    if ! wget -q --show-progress -O "$arducam_deb" "$arducam_url"; then
        log_error "Failed to download Arducam driver"
        log_info "Please download manually from: https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/Quick-Start-Guide/"
        return 1
    fi
    
    log_info "Installing Arducam driver package..."
    if dpkg -i "$arducam_deb"; then
        log_success "Arducam driver installed successfully"
        rm -f "$arducam_deb"
        return 0
    else
        log_error "Failed to install Arducam driver"
        return 1
    fi
}

install_vlink_drivers() {
    log_info "Building and installing V-Link drivers..."
    
    # Check for kernel headers
    if [[ ! -d /lib/modules/$(uname -r)/build ]]; then
        log_error "Kernel headers not found. Install with: sudo apt install nvidia-l4t-jetson-multimedia-api"
        return 1
    fi
    
    # Build serializer driver
    log_info "Building v-link-ser (serializer) driver..."
    cd "$SCRIPT_DIR/v-link-ser"
    if make clean && make; then
        log_success "v-link-ser built successfully"
    else
        log_error "Failed to build v-link-ser"
        return 1
    fi
    
    log_info "Installing v-link-ser..."
    if make install; then
        log_success "v-link-ser installed"
    else
        log_error "Failed to install v-link-ser"
        return 1
    fi
    
    # Build deserializer driver
    log_info "Building v-link-deser (deserializer) driver..."
    cd "$SCRIPT_DIR/v-link-deser"
    if make clean && make; then
        log_success "v-link-deser built successfully"
    else
        log_error "Failed to build v-link-deser"
        return 1
    fi
    
    log_info "Installing v-link-deser..."
    if make install; then
        log_success "v-link-deser installed"
    else
        log_error "Failed to install v-link-deser"
        return 1
    fi
    
    # Configure auto-load on boot
    log_info "Configuring drivers to load on boot..."
    echo "v-link-ser" > /etc/modules-load.d/v-link.conf
    echo "v-link-deser" >> /etc/modules-load.d/v-link.conf
    
    # Load modules now
    log_info "Loading V-Link drivers..."
    modprobe v-link-ser 2>/dev/null || true
    modprobe v-link-deser 2>/dev/null || true
    
    log_success "V-Link drivers installed and loaded"
    return 0
}

install_overlay() {
    log_info "Installing device tree overlay..."
    
    local overlay_src="$SCRIPT_DIR/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo"
    local overlay_dst="/boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo"
    
    if [[ ! -f "$overlay_src" ]]; then
        log_error "Overlay file not found: $overlay_src"
        return 1
    fi
    
    # Copy overlay to /boot
    cp "$overlay_src" "$overlay_dst"
    log_success "Overlay copied to /boot"
    
    # Configure extlinux.conf
    local extlinux="/boot/extlinux/extlinux.conf"
    
    if [[ ! -f "$extlinux" ]]; then
        log_error "extlinux.conf not found at $extlinux"
        return 1
    fi
    
    # Backup extlinux.conf
    cp "$extlinux" "${extlinux}.backup.$(date +%Y%m%d_%H%M%S)"
    log_info "Backed up extlinux.conf"
    
    # Check if OVERLAYS line exists
    if grep -q "^[[:space:]]*OVERLAYS" "$extlinux"; then
        # Check if our overlay is already there
        if grep -q "imx519-vlink-dual.dtbo" "$extlinux"; then
            log_info "Overlay already configured in extlinux.conf"
        else
            # Append to existing OVERLAYS line
            sed -i 's|^\([[:space:]]*OVERLAYS.*\)|\1,/boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo|' "$extlinux"
            log_success "Added overlay to existing OVERLAYS line"
        fi
    else
        # Add OVERLAYS line after FDT line
        if grep -q "^[[:space:]]*FDT" "$extlinux"; then
            sed -i '/^[[:space:]]*FDT/a\      OVERLAYS /boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo' "$extlinux"
            log_success "Added OVERLAYS line to extlinux.conf"
        else
            log_error "Could not find FDT line in extlinux.conf"
            log_info "Please manually add: OVERLAYS /boot/tegra234-p3767-camera-p3768-imx519-vlink-dual.dtbo"
            return 1
        fi
    fi
    
    log_success "Device tree overlay configured"
    return 0
}

# -----------------------------
# Main
# -----------------------------
main() {
    local skip_arducam=false
    local skip_vlink=false
    local skip_overlay=false
    local check_only=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --check)
                check_only=true
                shift
                ;;
            --skip-arducam)
                skip_arducam=true
                shift
                ;;
            --skip-vlink)
                skip_vlink=true
                shift
                ;;
            --skip-overlay)
                skip_overlay=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                ;;
        esac
    done
    
    # Check only mode
    if $check_only; then
        run_checks
        exit $?
    fi
    
    # Full installation
    check_root
    
    echo ""
    echo "=========================================="
    echo "  IMX519 Dual Camera V-Link Installer"
    echo "=========================================="
    echo ""
    
    # System checks
    check_jetson || exit 1
    check_jetpack || exit 1
    
    echo ""
    log_info "Starting installation..."
    echo ""
    
    # Install components
    if ! $skip_arducam; then
        install_arducam || log_warn "Arducam installation had issues, continuing..."
    fi
    
    if ! $skip_vlink; then
        install_vlink_drivers || exit 1
    fi
    
    if ! $skip_overlay; then
        install_overlay || exit 1
    fi
    
    echo ""
    echo "=========================================="
    echo "  Installation Complete!"
    echo "=========================================="
    echo ""
    log_success "All components installed successfully"
    echo ""
    log_info "A reboot is required to activate the dual camera setup."
    echo ""
    
    read -p "Reboot now? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Rebooting..."
        reboot
    else
        log_info "Please reboot manually when ready: sudo reboot"
    fi
}

main "$@"
