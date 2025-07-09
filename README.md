# wrpps_driver
Linux / ROS Driver Software for WrPPS Sensor Board.

---

## Hardware Outline

WrSSR Single Board is a board with proximity sensors.

- Connector Socket
  - SH Connector
    - 4 pin
    - SM04B-SRSS-TB(LF)(SN) - JST
    - https://www.jst-mfg.com/product/index.php?series=231
- Sensors
  - Time-of-Flight (ToF) Ranging Sensor
    - VL53L0X - STMicroelectronics
      - https://www.st.com/ja/imaging-and-photonics-solutions/vl53l0x.html
      - Range: ≦ 2.0 [m]
  - Proximity and Ambient Light Sensor
    - VCNL4040 - VISHAY
      - https://www.vishay.com/ja/product/84274/
      - Range: ≦ 0.2 [m]
- Dimensions
  - Width: 18 [mm]
  - Length: 18 [mm]
  - Height: 4.7 [mm]
- Weight: 1.5 [g]

<br>

![WrPPS Single Board v1.1 Dimensions](doc/images/wrpps-single-board_dimensions_small.png)


## System Requirements

- Ubuntu 20.04 + ROS Noetic
- Arduino IDE 1.8.19

### Arduino

- Arduino Leonardo
- Arduino Uno R3
- Arduino nano
- Arduino nano Every


## Installation

### ROS Software

``` bash
mkdir -p ~/wrpps_ws/src
cd ~/wrpps_ws/src
git clone https://github.com/tork-a/wrpps_driver.git
cd ~/wrpps_ws
rosdep install -y -r --from-paths src --ignore src
catkin build
source ~/wrpps_ws/devel/setup.bash
```

### Arduino Sketch

https://github.com/pazeshun/jsk_apc/blob/test_wrpps_single_board/demos/sphand_ros/sphand_driver/arduino/test_wrpps_single_board/test_wrpps_single_board.ino


## How to Use

### Pin Asign

SH Connector<br> Pin No. | Arduino<br> Leonardo / Uno R3 | Arduino<br> nano / nano Every
:---: | :---: | :---:
1 | GND | GND
2 | 3.3V | 3.3V
3 | SDA | A4 (SDA)
4 | SCL | A5 (SCL)

### Launching Software

```
source ~/wrpps_ws/devel/setup.bash
roslaunch wrpps_driver wrpps_ros.launch
```

Press Ctrl-C to stop.

