# Package ROHand

Nodes for ROHand. One bus, i.e. one port need one node.

## 1. Clone

```BASH
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone ssh://git@github.com/oymotion/rohand_ros2_pkg
```

## 2. Prepare

Install pymodbus

```BASH
cd /path/to/workspace  # Should be ~/ros2_ws/src

# Create a vertual env for python
virtualenv -p python3 ./venv

# Make sure that colcon doesn’t try to build the venv
touch ./venv/COLCON_IGNORE

# Activate
source ./venv/bin/activate

# Install python module
python3 -m pip install pymodbus
```

Edit `~/bashrc` and add virtual env lib path to PYTHONPATH

```BASH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/venv/lib/python3.12/site-packages  # Modify python3.12 to your actual versioni
source ~/.bashrc
```

## 3. Compile

```BASH
cd /path/to/workspace
colcon build
```

## 4. Node rohand

ROHand node for ModBus-RTU or ModBus-RTU & SerialCtrl Dual Protocol. Please confirm protocol type in OHandSetting.
Listens to topic 'target_joint_state' and controls ROHand, reads current joint state and publish to 'current_joint_state'.

### 4.1 Topics

| Topic                 | Description                                                                              |
| --------------------- | ---------------------------------------------------------------------------------------- |
| "current_joint_state" | current joint state in message type JointState, frame_id in header distinguishes hand id |
| "target_joint_state"  | target joint state in message type JointState, frame_id in header distinguishes hand id  |

### 4.2 Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Insert USB-485 module to USB port then add permission to users
# Run following command every time you plug in your USB-485 module
sudo chmod o+rw /dev/ttyUSB0  # Modify ttyUSB0 to your actual device name

# Run node
ros2 run rohand rohand --ros-args -p port_name:="/dev/ttyUSB0" -p baudrate:=115200 -p hand_ids:=[2,3]  # Modify parameters according to your real case
```

## 5. Node rohand_serial

ROHand node for SerialCtrl Protocol, Dual Protocol is NOT supported. Please confirm protocol type in OHandSetting.
Listens to topic 'target_joint_state' and controls ROHand, reads current joint state and publish to 'current_joint_state'.

### 5.1 Topics

| Topic                 | Description                                                                              |
| --------------------- | ---------------------------------------------------------------------------------------- |
| "current_joint_state" | current joint state in message type JointState, frame_id in header distinguishes hand id |
| "target_joint_state"  | target joint state in message type JointState, frame_id in header distinguishes hand id  |
| "finger_state"        | finger status in message type UInt8MultiArray                                            |

### 5.2 Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Insert USB-485 module to USB port then add permission to users
# Run following command every time you plug in your USB-485 module
sudo chmod o+rw /dev/ttyUSB0  # Modify ttyUSB0 to your actual device name

# Run node
ros2 run rohand rohand_serial --ros-args -p port_name:="/dev/ttyUSB0" -p baudrate:=115200 -p hand_ids:=[2,3]  # Modify parameters according to your real case
```

Finger status code:

| Name                 | Code  | Description                |
| -------------------- | :---: | -------------------------- |
| STATUS_OPENING       |   0   | Spreading                  |
| STATUS_CLOSING       |   1   | Grasping                   |
| STATUS_POS_REACHED   |   2   | Position reached stop      |
| STATUS_OVER_CURRENT  |   3   | Current protection stop    |
| STATUS_FORCE_REACHED |   4   | Force control reached stop |
| STATUS_STUCK         |   5   | Motor stuck stop           |

## 6. Node rohand_teleop

Reads keys to modify target joint angles, then publish to 'target_joint_state'.

### 6.1 Topics

| Topic                | Description                                                                             |
| -------------------- | --------------------------------------------------------------------------------------- |
| "target_joint_state" | target joint state in message type JointState, frame_id in header distinguishes hand id |

### 6.2 Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Run node
ros2 run rohand rohand_teleop --ros-args -r rohand_teleop_node/target_joint_states:=/rohand_node/target_joint_states -p hand_id:=2  # Modify parameters according to your real case
```

Press following keys to operate:

| key | Description            |
| --- | ---------------------- |
| q   | quit                   |
| a   | thumb bends by step    |
| z   | thumb relaxes by step  |
| s   | index bends by step    |
| x   | index relaxes by step  |
| d   | middle bends by step   |
| c   | middle relaxes by step |
| f   | ring bends by step     |
| v   | ring relaxes by step   |
| g   | little bends by step   |
| b   | little relaxes by step |
| h   | thumb rotation +step   |
| n   | thumb rotation -step   |

Step is range / 10.
