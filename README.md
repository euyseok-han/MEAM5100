# Dual Motor Wi-Fi RC Car (ESP32-C3)

Web-controlled differential drive RC car using an **ESP32-C3**, dual DC motors with encoders, and a **PID-based speed + steering controller** with a live dashboard in the browser.

This project was developed as part of **MEAM 5100** and extends a single-motor PID lab into a fully controllable two-motor RC platform with:

- Web-based control (no separate transmitter)
- Differential drive steering
- Real-time speed feedback from encoders
- On-the-fly PID tuning from the browser

---

## Features

- ✅ **Wi-Fi Control**  
  Control base speed and steering directly from a web UI hosted on the ESP32-C3.

- ✅ **Dual Motor Differential Drive**  
  Independent left/right wheel control for forward, reverse, turning, and spin-in-place behaviors.

- ✅ **Encoder-Based Speed Feedback**  
  Uses quadrature encoders on both wheels to compute RPM in real time.

- ✅ **PID Speed Control**  
  Separate PID controllers for left and right motors to track target wheel speeds.

- ✅ **Live Web Dashboard**
  - Speed & steering sliders
  - Direction indicator
  - Left/right target vs. actual speeds
  - PWM output & encoder readings
  - Simple historical speed plot

- ✅ **PID Tuning in Browser**  
  Adjust `Kp`, `Ki`, `Kd` via the web interface without reflashing firmware.

---

## Repository Structure

```bash
MEAM5100/
└── single_motor_test_updated/
    ├── single_motor_test_updated.ino   # Main ESP32-C3 firmware
    └── website.h                       # Embedded HTML/CSS/JS UI
