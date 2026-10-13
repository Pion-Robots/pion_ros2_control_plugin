# cybergear_ros2

This repository is for controlling CyberGear servo motors with ROS 2.
Currently, it includes the following packages:

+ [cybergear_driver_core](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_driver_core)

  This package provides manipulation of CyberGear's CAN frames independently of ROS 2.

+ [cybergear_driver_msgs](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_driver_msgs)

  This package provides messages used by the cybergear_socketcan_driver.

+ [cybergear_socketcan_driver](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_socketcan_driver)

  This package provides functionality to operate CyberGear using CAN frame messages.

+ [cybergear_maintenance_tools]

  To use this package, you have to use ros2 run and pass the parameters

  '''ros2 run cybergear_maintenance_tools change_cybergear_id_node --ros-args -p target_id:=126 -p device_id:=127'''

  In this example you change the ID of cybergear motor with id 127 to 126.
  Also remember to start the ros2_socketcan bridge and run ./can.sh with sudo!

  If for some reason this script stops working (like it has for us :/) you can do it manually using cansend!

  ```bash
  cansend can0 077D007F#00000001
  ```
  The command is structured as follows:
  - 07 - command id
  - 7D - new motor id
  - 00 - host id
  - 7F - current id
  - \#00000001 - Data field

+ [USB2CAN native]

  Setup the USB2CAN converter interface:

  ```
  sudo modprobe can
  sudo modprobe can_raw
  sudo modprobe can_dev

  sudo ip link set can0 type can bitrate 1000000

  sudo ip link set can0 up
  sudo ifconfig can0 txqueuelen 100
  ```

+ [USB2CAN without prompting password]

  ```
  sudo visudo -f /etc/sudoers.d/can-setup
  ```

  and add:

  ```
  # Allow passwordless sudo for CAN interface configuration
  $USER ALL=(ALL) NOPASSWD: /sbin/modprobe can
  $USER ALL=(ALL) NOPASSWD: /sbin/modprobe can_raw
  $USER ALL=(ALL) NOPASSWD: /sbin/modprobe can_dev
  $USER ALL=(ALL) NOPASSWD: /sbin/ip link set can_* *
  $USER ALL=(ALL) NOPASSWD: /sbin/ifconfig can_* *
  ```

+ Set Can Bus Persistent ID

  Get serial id
  ```
  udevadm info -a -p /sys/class/net/can0 | grep serial
  ```

  example output:
  ```
  ATTRS{serial}=="0041002F5034570520323530"
  ATTRS{serial}=="0000:00:14.0"
  ```

  Get idVendor and idProduct:

  ```
  udevadm info -a -p /sys/class/net/can4 | grep -E 'idVendor|idProduct'
  ```

  example output

  ```
  ATTRS{idProduct}=="606f"
  ATTRS{idVendor}=="1d50"
  ATTRS{idProduct}=="4100"
  ATTRS{idVendor}=="8564"
  ATTRS{idProduct}=="0002"
  ATTRS{idVendor}=="1d6b"
  ATTRS{dbc_idProduct}=="0010"
  ATTRS{dbc_idVendor}=="1d6b^C
  ```

  then create udev rules file in:

  ```
  sudo vim /etc/udev/rules.d/99-usb2can-names.rules
  ```

  ```
  # Udev Rules for USB2CAN Native Devices (PID 606f / VID 1d50)

  SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="0041002F5034570520323530", NAME="can_dev"
  ```

  ```
  sudo udevadm control --reload-rules
  ```

  unplug and plug back

  ```
  ip a
  ```
