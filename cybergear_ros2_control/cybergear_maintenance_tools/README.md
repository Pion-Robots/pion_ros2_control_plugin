# cybergear_maintenance_tools

Command line tools to inspect and configure MI CyberGear actuators on a CAN bus.

## Nodes

| Node | Executable | Purpose |
| --- | --- | --- |
| `SearchCybergearIdNode` | `search_cybergear_id_node` | Scan a bus and list every device id that answers |
| `ChangeCybergearIdNode` | `change_cybergear_id_node` | Change the device id of a single actuator |
| `SetZeroCybergearIdNode` | `set_zero_cybergear_id_node` | Set the mechanical zero of a single actuator |

All nodes open the SocketCAN interface directly (via `ros2_socketcan`), so **no
`ros2_socketcan` bridge node is required** — pass the Linux CAN interface name in
`can_bus`, the same value used for `can_interface` in the `cybergear_ros2_control`
xacro.

## search_cybergear_id_node

Sweeps device ids `1..255` with the CyberGear *get device id* (INFO) frame, collects
the ids that reply, and prints a summary before exiting.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `can_bus` | string | `can0` | SocketCAN interface name to scan (e.g. `can_left_arm`) |
| `send_frequency` | double | `64.0` | Scan frame rate in Hz |
| `wait_recive_can_frame` | double | `0.1` | Extra time in seconds to wait for replies after the sweep |

### Usage

```bash
ros2 run cybergear_maintenance_tools search_cybergear_id_node --ros-args -p can_bus:=can_left_arm
```

```
[INFO] [search_cybergear_id]: Scanning CyberGear devices on can_left_arm
[INFO] [search_cybergear_id]: Found CyberGear device: 2 on can_left_arm
[INFO] [search_cybergear_id]: Found CyberGear device: 4 on can_left_arm
[INFO] [search_cybergear_id]: Found CyberGear device: 6 on can_left_arm
[INFO] [search_cybergear_id]: Found CyberGear device: 8 on can_left_arm
[INFO] [search_cybergear_id]: Found 4 CyberGear device(s) on can_left_arm: 2, 4, 6, 8
```

Scan every bus of the Pion robot:

```bash
for bus in can_left_arm can_right_arm can_head can_left_leg can_right_leg; do
  ros2 run cybergear_maintenance_tools search_cybergear_id_node --ros-args -p can_bus:="$bus"
done
```

### Expected ids on the Pion humanoid

| Bus | Device ids |
| --- | --- |
| `can_right_arm` | 1, 3, 5, 7 |
| `can_left_arm` | 2, 4, 6, 8 |
| `can_right_leg` | 9, 11, 13, 15, 17, 19 |
| `can_left_leg` | 10, 12, 14, 16, 18, 20 |
| `can_head` | 21, 22 |

## change_cybergear_id_node

Sends a *change device id* frame to one actuator on the given bus and reports
whether the actuator answered with its new id.

> Connect only the actuator being reconfigured. The target id must not collide
> with another device on the bus, and the new id is persistent.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `can_bus` | string | `can0` | SocketCAN interface the actuator is connected to |
| `primary_id` | int | `0` | Host (master) CAN id |
| `device_id` | int | `127` | Current id of the actuator |
| `target_id` | int | `127` | New id to write |
| `send_frequency` | double | `64.0` | Frame rate in Hz |
| `wait_recive_can_frame` | double | `0.1` | Time in seconds to wait for the reply |

### Usage

```bash
ros2 run cybergear_maintenance_tools change_cybergear_id_node --ros-args \
  -p can_bus:=can_left_arm \
  -p device_id:=127 \
  -p target_id:=2
```

```
[INFO] [change_cybergear_id]: Start change_cybergear_id
[INFO] [change_cybergear_id]: CAN bus: can_left_arm
[INFO] [change_cybergear_id]: Primary ID: 0
[INFO] [change_cybergear_id]: Device ID: 127
[INFO] [change_cybergear_id]: Target ID: 2
[INFO] [change_cybergear_id]: Found new CyberGear device: 2 on can_left_arm
[INFO] [change_cybergear_id]: Changed device id 127 -> 2 on can_left_arm
[INFO] [change_cybergear_id]: Finish change_cybergear_id
```

Verify the result with `search_cybergear_id_node` on the same bus.

## set_zero_cybergear_id_node

Sets the mechanical zero of one actuator. It reads the mechanical position
(`mechPos`, RAM parameter `0x7019`) before sending the zeroing frame, waits for
the actuator to settle, then reads it back so you can confirm the new reference.

> The actuator keeps the zero position persistently. Make sure the joint is held
> in the pose you want to become zero before running this.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `can_bus` | string | `can0` | SocketCAN interface the actuator is connected to |
| `primary_id` | int | `0` | Host (master) CAN id |
| `device_id` | int | `127` | Device id of the actuator to zero |
| `wait_recive_can_frame` | double | `0.5` | Time in seconds to wait for each position reply |
| `wait_after_zeroing` | double | `0.2` | Settling time in seconds between zeroing and the read back |

### Usage

```bash
ros2 run cybergear_maintenance_tools set_zero_cybergear_id_node --ros-args \
  -p can_bus:=can_left_arm \
  -p device_id:=2
```

```
[INFO] [set_zero_cybergear_id]: Start set_zero_cybergear_id
[INFO] [set_zero_cybergear_id]: CAN bus: can_left_arm
[INFO] [set_zero_cybergear_id]: Primary ID: 0
[INFO] [set_zero_cybergear_id]: Device ID: 2
[INFO] [set_zero_cybergear_id]: Position before zeroing: 0.0139337 rad
[INFO] [set_zero_cybergear_id]: Sent zero position command to device 2
[INFO] [set_zero_cybergear_id]: Position after zeroing: -0.000170443 rad
[INFO] [set_zero_cybergear_id]: Finish set_zero_cybergear_id
```

## Troubleshooting

- **`No CyberGear device found`** — wrong interface name, bus not terminated or
  unpowered actuators. Confirm traffic with `candump can_left_arm`.
- **Fewer ids than expected** — increase `wait_recive_can_frame` or lower
  `send_frequency` to give slow devices more time to answer.
- Stop `ros2_control` before changing ids; both processes writing to the same
  bus can interleave frames.
