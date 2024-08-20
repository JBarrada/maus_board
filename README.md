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

![image](https://github.com/user-attachments/assets/b86efd99-d0ee-48d6-b2f3-cb1050980544)

Video:
https://www.youtube.com/watch?v=Ug7xEspHlwM


### Disclaimer
I am only doing this as a hobby. Please use the code and hardware with caution.

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
