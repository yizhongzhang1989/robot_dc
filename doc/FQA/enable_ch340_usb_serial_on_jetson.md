# ✅ Enabling CH340 USB-to-Serial Support on NVIDIA Jetson (L4T R36.x)

This guide provides a complete step-by-step solution to enable the CH340 USB-to-Serial adapter on Jetson devices (e.g., Jetson Orin Nano) running Jetson Linux (L4T R36.x), **by compiling only the required kernel module `ch341.ko`** from source.

---

## 🧭 Problem Summary

CH340-based USB-to-Serial adapters may not function by default on Jetson devices. Symptoms include:

- `lsusb` shows CH340 device (e.g., `1a86:7523`), but:
- No `/dev/ttyUSB*` appears
- `dmesg` shows CH340 detection and then disconnection
- `lsmod | grep ch341` returns nothing
- Even after hotplugging, device disappears or gets claimed by `brltty` or nothing

---

## 🔍 System Check

### 1. Verify Your Jetson L4T Version

```bash
head -n 1 /etc/nv_tegra_release
````

Example output:

```
# R36 (release), REVISION: 4.3, GCID: 37633192, BOARD: t186ref
```

### 2. Check Kernel Version

```bash
uname -r
```

Example output:

```
5.10.104-tegra
```

---

## ⚙️ Enable CH340 Driver Support in Kernel

### 3. Install Required Packages

```bash
sudo apt update
sudo apt install -y build-essential bc flex bison libssl-dev libncurses5-dev
```

---

### 4. Download and Extract Kernel Sources

#### a. Download public sources

Download `public_sources.tbz2` from the [Jetson Linux Archive](https://developer.nvidia.com/embedded/jetson-linux-archive) that matches your L4T version.

#### b. Extract the kernel source

```bash
mkdir -p ~/jetson-kernel
cd ~/jetson-kernel
tar -xvf ~/Downloads/public_sources.tbz2
cd Linux_for_Tegra/source
tar -xvf kernel_src.tbz2
cd ../kernel/kernel-jammy-src
```

*Note: directory name might be `kernel-focal-src` depending on your base OS.*

---

### 5. Prepare Kernel Config

#### a. Use your current kernel config

```bash
zcat /proc/config.gz > .config
```

#### b. Enable CH341 driver via `menuconfig`

```bash
make menuconfig
```

Navigate to:

```
Device Drivers  --->
  [*] USB support  --->
    [*] USB Serial Converter support --->
      <M> USB CH341 single port serial driver
```

Make sure **CH341** is set to `M` (module), **not built-in (`*`)** or disabled.

Save and exit.

---

## 🏗️ Build and Install the `ch341.ko` Module

### 6. Build Kernel Modules Only

```bash
make -j$(nproc) modules
```

This builds all external kernel modules including `ch341.ko`.

---

### 7. Locate and Install `ch341.ko`

```bash
find . -name ch341.ko
```

Example output:

```
./drivers/usb/serial/ch341.ko
```

Then install it:

```bash
sudo cp drivers/usb/serial/ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo depmod
sudo modprobe ch341
```

---

## 🔌 Plug and Test the CH340 Adapter

### 8. Plug in the Adapter

Once connected, verify with:

```bash
dmesg | tail -n 20
```

Expected log snippet:

```
ch341 1-2.3:1.0: ch341-uart converter detected
usb 1-2.3: ch341-uart converter now attached to ttyUSB0
```

Then:

```bash
ls /dev/ttyUSB*
```

Expected output:

```
/dev/ttyUSB0
```

---

## ✅ Optional: Load Automatically on Boot

If the module works, enable auto-loading at boot:

```bash
echo ch341 | sudo tee -a /etc/modules
```

---

## 🧪 Optional: Test with minicom

```bash
sudo apt install minicom
minicom -D /dev/ttyUSB0
```

---

## ✅ Summary Table

| Step                            | Description                               |
| ------------------------------- | ----------------------------------------- |
| `uname -r` / `nv_tegra_release` | Check L4T and kernel versions             |
| `apt install ...`               | Install required build packages           |
| Extract sources                 | `public_sources.tbz2` → `kernel_src.tbz2` |
| `.config`                       | Use running kernel config                 |
| `make menuconfig`               | Enable `ch341` as module                  |
| `make modules`                  | Build only external modules               |
| `cp`, `depmod`, `modprobe`      | Install and load module                   |
| `ls /dev/ttyUSB0`               | Verify working device                     |
| `echo ch341 >> /etc/modules`    | Enable on boot                            |

---

## 📚 References

* [Jetson Linux Archive (Download Sources)](https://developer.nvidia.com/embedded/jetson-linux-archive)
* [Jetson Kernel Customization Guide (R36.x)](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Kernel/KernelCustomization.html)
* [NVIDIA Forum CH340 Discussion](https://forums.developer.nvidia.com/t/issue-with-ch340-usb-to-serial-converter-not-creating-device-files-on-jetson-orin-nano-super/326022)
