# Sotong_Game_STM32

This project implements a simple Red Light, Green Light + Catch Game on the STM32L4S5I-IOT01 board using the onboard sensors.
The game uses accelerometer, gyroscope, magnetometer, pressure, humidity, and temperature sensors to determine player movement and trigger events.


Game Modes
1) Red Light, Green Light
    Reads accelerometer & gyro to detect movement.
   
    Player is “caught” if motion exceeds thresholds during Red Light

3) Catch and Run
    Uses magnetometer readings.
   
    LED blinking speed adjusts based on magnetic magnitude.
   
    Player can “escape” with a quick button tap
