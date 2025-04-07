# ESP32 WiFi Drone / FC

This project is a minimalist flight controller firmware for the ESP32, developed using the ESP-IDF framework in C/C++. The firmware utilizes the Digital Motion Processor (DMP) found in TDK IMU modules such as the MPU6050, MPU6500, and MPU9250.

It features PID-based control loops running at 200Hz to manage roll, pitch, and yaw angles. The project is also designed to support altitude stabilization using pressure sensors like the BMP280 or BMP388.

    The main branch contains the initial prototype firmware.

    Development for version v1.1 is currently in progress.

In addition to the firmware, I have also designed PCB layouts for both the mini drone and its flight controller. These designs can be found in the hardware/ directory.

follow my linkedin post for more info. [vist](https://www.linkedin.com/posts/nuran-nimdinu-5639562a8_esp32-dronedevelopment-pcbdesign-activity-7313055099948343298-wLOK?utm_source=share&utm_medium=member_desktop&rcm=ACoAAEot7JsB93VQnTDrcylimKZKjH7dZPS82M8)
