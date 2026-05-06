# 🦑 Sotong Game: STM32 Interactive Sensor Game

![STM32](https://img.shields.io/badge/STM32-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)

An embedded systems project playfully inspired by "Squid Game" (or *Sotong Game*!), implemented on the **STM32L4S5I-IOT01** development board. This project transforms the microcontroller into an interactive physical game using its onboard suite of sensors to track real-world player movements.

## 🎮 Game Modes

### 1. Red Light, Green Light 🚦
A test of stillness and reflexes. 
- **How it works:** The system reads real-time data from the onboard **accelerometer** and **gyroscope**.
- **The Catch:** During the "Red Light" phase, any physical movement exceeding the programmed threshold will result in the player being "caught". Hold the board perfectly still to survive!

### 2. Catch and Run 🏃‍♂️🧲
A game of proximity and timing.
- **How it works:** This mode utilizes the onboard **magnetometer**. 
- **The Catch:** The onboard LED blinking speed dynamically adjusts based on the ambient magnetic field magnitude (e.g., when a magnet is brought closer to the board).
- **The Escape:** The player can successfully "escape" by registering a quick, precise button tap before the timer runs out.

## 🛠️ Hardware & Software

- **Board:** [B-L4S5I-IOT01A Discovery kit](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html)
- **Language:** C (Bare-metal / HAL)
- **IDE:** STM32CubeIDE
- **Sensors Utilized:** 
  - Accelerometer & Gyroscope (LSM6DSL)
  - Magnetometer (LIS3MDL)
  - Environmental Sensors (Pressure, Humidity, Temperature)

## 🚀 Getting Started

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Mayuresh2706/Sotong_Game_STM32.git
