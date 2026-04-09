#!/bin/bash
# Build script for ESP32 Zephyr firmware

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
FIRMWARE_DIR="${SCRIPT_DIR}/firmware"
ZEPHYR_DIR="${SCRIPT_DIR}/zephyr"

# Set environment variables
export ZEPHYR_BASE="${ZEPHYR_DIR}"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk-0.16.8

# Parse arguments
BOARD="${1:-esp32_devkitc_wroom/esp32/procpu}"
ACTION="${2:-build}"

echo "=== Zephyr ESP32 Build Script ==="
echo "Board: $BOARD"
echo "Action: $ACTION"
echo "Build directory: $BUILD_DIR"
echo ""

case $ACTION in
  clean)
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    ;;
  distclean)
    echo "Cleaning all build artifacts..."
    rm -rf "$BUILD_DIR" "${ZEPHYR_DIR}/build"
    ;;
  build|configure)
    echo "Configuring CMake..."
    cmake -S "$FIRMWARE_DIR" -B "$BUILD_DIR" \
      -GNinja \
      -DBOARD="$BOARD" \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    
    if [ "$ACTION" = "configure" ]; then
      echo "Configuration complete."
    else
      echo "Building firmware..."
      cd "$BUILD_DIR"
      ninja
      echo ""
      echo "✅ Build complete!"
      echo "Firmware image: $BUILD_DIR/zephyr/zephyr.elf"
      echo "ESP32 image: $BUILD_DIR/zephyr/zephyr.bin"
    fi
    ;;
  rebuild)
    echo "Cleaning and rebuilding..."
    rm -rf "$BUILD_DIR"
    cmake -S "$FIRMWARE_DIR" -B "$BUILD_DIR" \
      -GNinja \
      -DBOARD="$BOARD"
    cd "$BUILD_DIR"
    ninja
    echo ""
    echo "✅ Rebuild complete!"
    ;;
  flash)
    if [ ! -f "$BUILD_DIR/zephyr/zephyr.bin" ]; then
      echo "❌ Firmware not found. Run 'build' first."
      exit 1
    fi
    echo "Flashing firmware to ESP32..."
    python3 /opt/toolchains/modules/hal/espressif/tools/esptool_py/esptool.py \
      -p /dev/ttyUSB0 \
      -b 115200 \
      --before default_reset \
      --after hard_reset \
      --chip esp32 write_flash \
      --flash_mode dio \
      --flash_size detect \
      --flash_freq 40m \
      0x1000 "$BUILD_DIR/zephyr/zephyr.bin"
    echo "✅ Flash complete!"
    ;;
  help|--help|-h)
    echo "Usage: $0 [BOARD] [ACTION]"
    echo ""
    echo "BOARD (default: esp32_devkitc_wroom/esp32/procpu)"
    echo "  esp32_devkitc_wroom/esp32/procpu"
    echo "  esp32_devkitc_wroom/esp32/secpu"
    echo ""
    echo "ACTION (default: build)"
    echo "  configure  - Configure CMake only"
    echo "  build      - Configure and build"
    echo "  rebuild    - Clean and rebuild"
    echo "  clean      - Remove build directory"
    echo "  distclean  - Remove all build artifacts"
    echo "  flash      - Flash to ESP32 (requires connection on /dev/ttyUSB0)"
    echo "  help       - Show this message"
    ;;
  *)
    echo "❌ Unknown action: $ACTION"
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
