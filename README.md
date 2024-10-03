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
sudo chmod o+rw /dev/ttyUSB0  # Modify ttyUSB0 to your actual device name
```

Edit `~/bashrc` and add virtual env lib path to PYTHONPATH 

```BASH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/venv/lib/python3.12/site-packages 	# Modify python3.12 to your actual versioni
source ~/.bashrc
```

## Node ROHand

### Topics

| Topic                 | Description                                                                              |
| --------------------- | ---------------------------------------------------------------------------------------- |
| "current_joint_state" | current joint state in message type JointState, frame_id in header distinguishes hand id |
| "target_joint_state"  | target joint state in message type JointState, frame_id in header distinguishes hand id  |

### Compile

```BASH
cd /path/to/workspace
colcon build
```

### Run

```BASH
# Prepare package
source /path/to/workspace/install/bash

# Run node
ros2 run rohand rohand --ros-args -r __node:=bus1 -p port_name:="/dev/ttyUSB0" -p baudrate:=115200 -p hand_ids:=[2,3]
```

