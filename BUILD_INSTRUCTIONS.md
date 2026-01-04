# TSDZ8 Firmware Build Instructions (Debian Linux Console)

This guide explains how to compile the TSDZ8 firmware on Debian Linux using only the console/terminal.

## Prerequisites Installation

Install required packages:
```bash
sudo apt update
sudo apt install wget curl git build-essential cmake ninja-build python3
```

## Method 1: ModusToolbox (Infineon's Official Toolchain) - Recommended

### 1. Download and Install ModusToolbox

```bash
cd ~/Downloads
wget https://www.infineon.com/dgdl/Infineon-ModusToolbox_3.2.0_Linux_64-bit-Software-v03_02_00-EN.zip
unzip ModusToolbox_3.2.0_Linux_64-bit.zip
chmod +x ModusToolbox_3.2.0_Linux_64-bit.run
./ModusToolbox_3.2.0_Linux_64-bit.run
```

### 2. Setup Environment

Add to your PATH:
```bash
echo 'export PATH=$PATH:~/ModusToolbox/tools_3.2/modus-shell/bin' >> ~/.bashrc
source ~/.bashrc
```

### 3. Compile the Firmware

```bash
cd /home/myself/projects/hobby/OSF_860C
modus-shell
make getlibs
```

**For Production/Motor Controller Use:**
```bash
make build CONFIG=Release
```

**For Development/Testing:**
```bash
make build CONFIG=Debug
```

## Method 2: GCC ARM Toolchain (Alternative)

If ModusToolbox installation fails:

### 1. Install ARM GCC Toolchain

```bash
sudo apt install gcc-arm-none-eabi
```

### 2. Try Direct Compilation

```bash
cd /home/myself/projects/hobby/OSF_860C
make help  # Check available targets
make all   # Try direct compilation
```

## Build Configuration Guide

| Configuration | Use Case | Features |
|---------------|----------|----------|
| `CONFIG=Release` | **Motor Controller Flashing** | Optimized, smaller, faster |
| `CONFIG=Debug` | Development/Testing | Debug symbols, larger, slower |

⚠️ **Important**: Always use `CONFIG=Release` for actual motor controller flashing!

## Output Files

After successful compilation, find the firmware file:

**Release Build:**
```
build/tsdz8_for_GPIO_TEST/Release/tsdz8-fw-YYYYMMDD_HHMM.hex
```

**Debug Build:**
```
build/tsdz8_for_GPIO_TEST/Debug/tsdz8-fw-YYYYMMDD_HHMM.hex
```

*Note: YYYYMMDD_HHMM is automatically generated timestamp (e.g., 20250821_0009)*

## Troubleshooting

### If Compilation Fails:

1. **Check Makefile contents:**
```bash
cat Makefile | head -20
```

2. **Look for build scripts:**
```bash
find . -name "*.mk" -o -name "build.sh" -o -name "compile.sh"
```

3. **Check for CMake files:**
```bash
find . -name "CMakeLists.txt"
```

4. **Clean and retry:**
```bash
make clean
make getlibs
make build CONFIG=Release
```

### Common Issues:

- **Missing dependencies**: Install additional packages if compilation fails
- **Path issues**: Ensure ModusToolbox tools are in PATH
- **Permission errors**: Check file permissions in project directory

## Flashing the Firmware

After successful compilation:

1. Use the generated `.hex` file with Segger J-Link tools
2. Follow the hardware connection guide in the main README
3. **Important**: Never power motor with battery while J-Link is connected

## Notes

- This firmware is specifically for TSDZ8 motors with XMC1302 microprocessor
- Requires compatible 860C/SW102 display firmware
- Release builds are production-ready for motor controller flashing
- Debug builds include extra debugging features for development

For hardware setup and flashing instructions, refer to the main [README.md](README.md).