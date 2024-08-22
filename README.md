> [!NOTE]
> This repository is just for the PCB, firmware and related driver code. The code relating to the control, navigation, SLAM, etc of my specific implementation for the car will remain private, for now. This project is intended to make it easier and cheaper for people to get started developing their own autonomous vehicle code.  

## MAUS Board
This project was developed as a means to create a very inexpensive F1Tenth car. It is a Raspberry Pi “hat” that connects various peripherals required for a functioning autonomous vehicle.

In its current state it has connections for:
- Serial LiDAR device (FHL-LD19)
- I2C IMU (MPU6050)
- Neopixel RGB LEDs (two LEDs onboard)
- Standard servo and ESC connectors

In addition to that, various 5V and 3.3V power connections, spare I2C pads, an external RGB strip connection, and test points for any unused ESP32 pins.

> [!WARNING]
> In order to make this board useful to the majority of people using ROS, I would need to provide a ROS driver for this board. I know next to nothing about ROS and I would appreciate any help writing a ROS driver for this. See source/example.cpp for how to communicate with the board.

![image](https://github.com/user-attachments/assets/33a185bd-26c8-4e59-aa50-dd889c8ff06f)

![image](https://github.com/user-attachments/assets/db537b5e-f5b3-458c-80c6-88dc17858f35)

![image](https://github.com/user-attachments/assets/b86efd99-d0ee-48d6-b2f3-cb1050980544)

Video:
https://www.youtube.com/watch?v=Ug7xEspHlwM


### Disclaimer
I am only doing this as a hobby, and this project is very much DIY and not a complete solution. Please use the code and hardware with caution.

---

### TODOs
- ROS driver
- Bidirectional DShot instead of seperate ESC telemetry wire

---

### Supported ESCS
Any ESC that supports an external telemetry wire will work. It also must have a 5V BEC that can provide at least 3A to power the Pi and board.

Recommended ESC: https://www.amazon.com/dp/B0CPXXPPFH

---

### Raspberry Pi Setup:
Note: I’ve only tested this on the Raspberry Pi 4 since it has additional UART interfaces that can be enabled. This is so that the ESP32 and LiDAR can communicate with the Pi on separate interfaces. This will not work on a Pi3 and I don’t know if it will work on a Pi5.

#### Assuming a fresh install of Raspbian lite:
`sudo apt update`

`sudo apt upgrade`

#### Enable the additional UARTs:
`sudo raspi-config`

Interface Options -> Serial Port -> Shell? No -> Enable Hardware? Yes

Exit -> Reboot? No

#### Modify config.txt
`sudo nano /boot/firmware/config.txt`

Add the following lines at the end, before the `enable_uart=1` line

`dtoverlay=uart2`

`dtoverlay=uart3`

Save and exit nano (ctrl+s, ctrl+x)

`sudo reboot`

---

### Running the example
Clone this repo to the Pi

CD into the source folder

`make example`

`./example`

--- 

### BOM
Here's a Bill Of Materials for the parts I used in my car:
| Part            | Link                                         | Price   |
|-----------------|----------------------------------------------|---------|
| Chassis         | https://www.amazon.com/gp/product/B0CN9Q8LMN | $170.00 |
| Raspberry Pi    | https://www.amazon.com/dp/B07TC2BK1X         |  $62.00 |
| LiDAR           | https://www.amazon.com/dp/B0B1V8D36H         |  $70.00 |
| IMU             | https://www.amazon.com/dp/B08FHXQ58X         |  $12.00 |
| Brushless Motor | https://www.amazon.com/gp/product/B0B1D3K8TP |  $23.00 |
| ESC             | https://www.amazon.com/gp/product/B0CPXXPPFH |  $50.00 |
| ESC Plugs       | https://www.amazon.com/gp/product/B08LL8HPHR |   $8.00 |
| MAUS Board      |                                              | ~$30?   |
| TOTAL           |                                              | ~$425   |

I still don't know what to price that MAUS board at, so $30 is a placeholder. But you can save a lot money if you already have some similar parts.

I chose that chassis because it was the cheapest I could find that was ready to run in 1/10 scale form factor. I also have CAD files for a custom mounting plate for all the electronics that you can laser cut/cnc. (TODO include that file). If you find a cheaper chassis or one that already has a brushless motor, please let me know so I can update the BOM. 

If your chassis already has a brushless motor, you don't need to buy the motor. And if you don't care about getting ESC telemetry (ERPM, battery voltage, etc) then you also don't need to buy any of the ESC related things and you can use the one that came with your chassis. Although having ERPM information can make localization and dead-reckoning better in your implementation.

If you do decide to get the ESC, it does not come with a battery connector, so you will have to solder on one that matches your battery. In addition to that you will need to solder on a wire to the telemetry pad on the ESC to connect to the "ESC TELEM" pin on the board.

The ESC runs on AM32 firmware which is actively maintained and provides some cool features that make it easier to use for this project. We will need to configure the ESC to send telemetry and respond to servo commands like a typical RC car ESC. ESCs running BLHeli32 firmware will also work as they have the same features and configuration but BLHeli is no longer maintained so I don't recommened it.

TODO WRITE THOSE INSTRUCTIONS 

---

### Contact
https://www.instagram.com/jus10barrada/

