# RC Plane Stabilization System with ESP32 & MPU6050

###  Overview  
This project is a **simple stabilization system** for an RC plane using an **ESP32 microcontroller** paired with an **MPU6050 IMU sensor**. It reads orientation data from the IMU and adjusts the servos in real-time to stabilize the plane. It also takes PWM inputs from a standard RC receiver for pilot control.

---

###  Hardware Components  
- **ESP32 Dev Board** — main controller  
- **MPU6050 (Gyro + Accelerometer)** — orientation sensing  
- **RC Receiver** — provides PWM input signals from the pilot transmitter  
- **Servos** — control surfaces (ailerons, elevator, rudder) actuated by ESP32 outputs  
- Power supply and wiring as per your plane setup

---

###  How It Works  
1. **IMU data capture:** The ESP32 reads raw gyro and accelerometer data from the MPU6050 over I2C.  
2. **Sensor fusion:** A simple complementary filter or Kalman filter estimates the plane’s current attitude (pitch, roll).  
3. **Control logic:** Compares desired pilot input from RC receiver PWM signals with current attitude.  
4. **Servo output:** Calculates corrections and sends PWM signals to servos to stabilize the plane and follow pilot commands.

---

###  Features  
- Real-time attitude estimation from MPU6050 data  
- PWM input reading from RC receiver channels  
- Servo PWM output generation via ESP32  
- Modular, easy to adapt for different planes and control surfaces

---

###  Wiring Overview  

| ESP32 Pin      | Connected To            | Notes                     |  
| -------------- | ----------------------- | ------------------------- |  
| GPIO 21 (SDA)  | MPU6050 SDA             | I2C data line             |  
| GPIO 22 (SCL)  | MPU6050 SCL             | I2C clock line            |   
| GPIO 34        | Servo 1 (e.g. aileron)  | PWM output                |  
| GPIO 35        | Servo 2 (e.g. rudder)   | PWM output                |  

*(Adjust pins based on your hardware setup)*

---

###  Installation & Setup  
1. Flash the ESP32 with the provided firmware (code in `code.cpp` folder).  
2. Connect MPU6050 to ESP32 via I2C (SDA/SCL).  
3. Connect RC receiver PWM outputs to ESP32 GPIOs configured as inputs.  
4. Connect servos to ESP32 PWM output pins.  
5. Power your setup carefully (check servo power requirements).  
6. Calibrate MPU6050 offsets if needed (code includes calibration routine).  
7. Test and tune PID/control parameters for smooth stabilization.

---


###  Futur Improvements  
- Add more advanced sensor fusion (Madgwick/Mahony algorithms)  
- Integrate fail-safe and signal loss detection  
- Support more control channels and surface mixing  
- Add telemetry data output for debugging  
- Optimize control loop timing for lower latency  

---

### References  
- [MPU6050 datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)  
 

---

**Fly safe and stay stable! ✈️**

