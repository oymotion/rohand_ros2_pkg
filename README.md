# Package ROHand

Nodes for ROHand. One bus, i.e. one port need one node.

## Clone 
```BASH
cd ~
mkdir -p ros2_ws/src 
cd ros2/src
git clone ssh://git@github.com/oymotion/rohand
```

## Prepare

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

# Insert USB-485 module to USB port then add permition to users
# Run following command every time you plug in your USB-485 module
sudo chmod o+rw /dev/ttyUSB0  # Modify ttyUSB0 to your actual device name
```

Edit `~/bashrc` and add virtual env lib path to PYTHONPATH 

```BASH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/venv/lib/python3.12/site-packages 	# Modify python3.12 to your actual versioni
source ~/.bashrc
```

## Compile

```BASH
cd /path/to/workspace
colcon build
```

## Node rohand

Listen to topic 'target_joint_state' and controls ROHand, reads current joint state and publish to 'current_joint_state'. 

### Topics

| Topic                 | Description                                                                              |
| --------------------- | ---------------------------------------------------------------------------------------- |
| "current_joint_state" | current joint state in message type JointState, frame_id in header distinguishes hand id |
| "target_joint_state"  | target joint state in message type JointState, frame_id in header distinguishes hand id  |

### Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Run node
ros2 run rohand rohand --ros-args -r __node:=bus1 -p port_name:="/dev/ttyUSB0" -p baudrate:=115200 -p hand_ids:=[2,3]
```
## Node rohand_teleop

Read keys to modify target joint angles, then publish to 'target_joint_state'. 

### Topics

| Topic                 | Description                                                                              |
| --------------------- | ---------------------------------------------------------------------------------------- |
| "target_joint_state"  | target joint state in message type JointState, frame_id in header distinguishes hand id  |

### Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Run node
ros2 run rohand rohand_teleop --ros-args -r rohand_teleop_node/target_joint_states:=/rohand_node/target_joint_states -p hand_id:=2  # Modify hand_id according to your real case
```

Press following keys to operate:

| key | Description                |
| --- | -------------------------- |
| q   | quit                       |
| a   | thumb bends by 1 degree    |
| z   | thumb relaxes by 1 degree  |
| s   | index bends by 1 degree    |
| x   | index relaxes by 1 degree  |
| d   | middle bends by 1 degree   |
| c   | middle relaxes by 1 degree |
| f   | ring bends by 1 degree     |
| v   | ring relaxes by 1 degree   |
| g   | little bends by 1 degree   |
| b   | little relaxes by 1 degree |
| h   | thumb rotation +1 degree   |
| n   | thumb rotation -1 degree   |
 
