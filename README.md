# Debix Smart RC Car — NXP Cup 2025

This project runs the **image processing** and **control logic** for the smart RC car using a **Debix Model B** with an **NXP i.MX 8M Plus** processor.  
It handles:
- Camera capture & processing (**OpenCV**)
- Pure Pursuit steering calculation
- TCP & Serial communication to the Teensy
- Optional TCP client to connect to your website for data/debug

---

## Project Structure

```
Proiect-NXP-CUP-2025-/
├── build/                # Build output for Debix C++ app
├── include/              # C++ header files and modules
├── src/                  # C++ source files
├── out/                  # (Optional) Generated output/logs
├── test/                 # Test code
├── Teensy-Code/          # Teensy firmware(separate build using Platformio)
├── VisionDebixTCP/       # Python TCP client
│   ├── TCPclient.py
├── .venv/                # Python virtual environment for TCP client
├── CMakeLists.txt        # C++ build configuration
├── debix-systemd-setup.txt # Systemd service setup instructions
└── README.md
```

---

## Requirements

**Debix Model B** running Linux  
**CMake** >= 3.15  
**g++** or Clang  
**OpenCV** installed  
**Python 3** (for TCP client)  
Teensy must be flashed separately with Platformio

---

## Build Instructions — C++ Debix App in command line

1. From the project root:
   cmake -S . -B build

2. Build the project:
   cmake --build build

3. The executable will be in `build/`:
   ./build/DEBIX

---

## Run Instructions

### ✅ Run C++ Debix App

1. Make sure:
   - USB camera is connected
   - Teensy is connected
   - ENABLE_TCP_SITE_DEBUG is 1 if you want web access / 0 if in race mode

2. Run your app:
   ./build/DEBIX

### ✅ Run Python TCP Client

1. Activate your Python virtual environment:
   source .venv/bin/activate

2. Go to your TCP client folder:
   cd VisionDebixTCP
   python TCPclient.py

---

## ⚡ Teensy Firmware

The `Teensy-Code/` folder holds your Teensy microcontroller code.  
Flash it separately using **PlatformIO**.

---

## ✅ Tips

- Use `config.h` for all kind of config settings.
- Check camera details with `v4l2-ctl --list-devices`.