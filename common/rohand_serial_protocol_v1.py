

import threading
import time
import serial


PROTOCOL_VERSION_MAJOR = 3


# OHand error codes

# Error codes for remote peer, protocol part
ERR_PROTOCOL_WRONG_LRC                       =  0x01



# Error codes for remote peer, command part
ERR_COMMAND_INVALID                          =  0x11
ERR_COMMAND_INVALID_BYTE_COUNT               =  0x12
ERR_COMMAND_INVALID_DATA                     =  0x13


# Error codes for remote peer, status part
ERR_STATUS_INIT                              =  0x21
ERR_STATUS_CALI                              =  0x22
ERR_STATUS_STUCK                             =  0x23


# Error codes for remote peer, operation result part
ERR_OP_FAILED                                =  0x31
ERR_SAVE_FAILED                              =  0x32


# API return values
HAND_RESP_HAND_ERROR                          = 0xFF           # device error, error call back will be called with OHand error codes listed above
HAND_RESP_SUCCESS                             = 0x00
HAND_RESP_TIMER_FUNC_NOT_SET                  = 0x01           # local error, timer function not set, call HAND_SetTimerFunction(...) first
HAND_RESP_INVALID_CONTEXT                     = 0x02           # local error, invalid context, NULL or send data function not set
HAND_RESP_TIMEOUT                             = 0x03           # local error, time out when waiting node response
HAND_RESP_INVALID_OUT_BUFFER_SIZE             = 0x04           # local error, out buffer size not matched to returned data
HAND_RESP_UNMATCHED_ADDR                      = 0x05           # local error, unmatched node id between returned and waiting
HAND_RESP_UNMATCHED_CMD                       = 0x06           # local error, unmatched command between returned and waiting
HAND_RESP_DATA_SIZE_TOO_BIG                   = 0x07           # local error, size of data to send exceeds the buffer size
HAND_RESP_DATA_INVALID                        = 0x08           # local error, data content invalid


# Sub-command for HAND_CMD_SET_CUSTOM
SUB_CMD_SET_SPEED   = (1 << 0)
SUB_CMD_SET_POS     = (1 << 1)
SUB_CMD_SET_ANGLE   = (1 << 2)
SUB_CMD_GET_POS     = (1 << 3)
SUB_CMD_GET_ANGLE   = (1 << 4)
SUB_CMD_GET_CURRENT = (1 << 5)
SUB_CMD_GET_FORCE   = (1 << 6)
SUB_CMD_GET_STATUS  = (1 << 7)



# Constants

MAX_PROTOCOL_DATA_SIZE = 64

# Protocol states
WAIT_ON_HEADER_0            = 0
WAIT_ON_HEADER_1            = 1
WAIT_ON_ADDRESSED_NODE_ID   = 2
WAIT_ON_OWN_NODE_ID         = 3
WAIT_ON_COMMAND_ID          = 4
WAIT_ON_BYTE_COUNT          = 5
WAIT_ON_DATA                = 6
WAIT_ON_LRC                 = 7


# Protocol byte name
NODE_ID_BYTE_NUM    = 0
OWN_ID_BYTE_NUM     = 1
CMD_ID_BYTE_NUM     = 2
DATA_CNT_BYTE_NUM   = 3
DATA_START_BYTE_NUM = 4


#
# OHand commands
#

# Chief GET commands
HAND_CMD_GET_PROTOCOL_VERSION     = 0x00  # Get protocol version, Please don't modify!
HAND_CMD_GET_FW_VERSION           = 0x01  # Get firmware version
HAND_CMD_GET_HW_VERSION           = 0x02  # Get hardware version, [HW_TYPE, HW_VER, BOOT_VER_MAJOR, BOOT_VER_MINOR]
HAND_CMD_GET_CALI_DATA            = 0x03  # Get calibration data
HAND_CMD_GET_FINGER_PID           = 0x04  # Get PID of finger
HAND_CMD_GET_FINGER_CURRENT_LIMIT = 0x05  # Get motor current limit of finger
HAND_CMD_GET_FINGER_CURRENT       = 0x06  # Get motor current of finger
HAND_CMD_GET_FINGER_FORCE_LIMIT   = 0x07  # Get force limit of finger
HAND_CMD_GET_FINGER_FORCE         = 0x08  # Get force of finger
HAND_CMD_GET_FINGER_POS_LIMIT     = 0x09  # Get absolute position limit of finger
HAND_CMD_GET_FINGER_POS_ABS       = 0x0A  # Get current absolute position of finger
HAND_CMD_GET_FINGER_POS           = 0x0B  # Get current logical position of finger
HAND_CMD_GET_FINGER_ANGLE         = 0x0C  # Get first joint angle of finger
HAND_CMD_GET_THUMB_ROOT_POS       = 0x0D  # Get preset position of thumb root, [0, 1, 2, 255], 255 as invalid
HAND_CMD_GET_FINGER_POS_ABS_ALL   = 0x0E  # Get current absolute position of all fingers
HAND_CMD_GET_FINGER_POS_ALL       = 0x0F  # Get current logical position of all fingers
HAND_CMD_GET_FINGER_ANGLE_ALL     = 0x10  # Get first joint angle of all fingers

# Auxiliary GET commands
HAND_CMD_GET_SELF_TEST_LEVEL      = 0x20  # Get self-test level state
HAND_CMD_GET_BEEP_SWITCH          = 0x21  # Get beep switch state
HAND_CMD_GET_BUTTON_PRESSED_CNT   = 0x22  # Get button press count
HAND_CMD_GET_UID                  = 0x23  # Get 96 bits UID
HAND_CMD_GET_BATTERY_VOLTAGE      = 0x24  # Get battery voltage
HAND_CMD_GET_USAGE_STAT           = 0x25  # Get usage stat


# Chief SET commands
HAND_CMD_RESET                    = 0x40  # Please don't modify
HAND_CMD_POWER_OFF                = 0x41  # Power off
HAND_CMD_SET_NODE_ID              = 0x42  # Set node ID
HAND_CMD_CALIBRATE                = 0x43  # Recalibrate hand
HAND_CMD_SET_CALI_DATA            = 0x44  # Set finger pos range & thumb pos set
HAND_CMD_SET_FINGER_PID           = 0x45  # Set PID of finger
HAND_CMD_SET_FINGER_CURRENT_LIMIT = 0x46  # Set motor current limit of finger
HAND_CMD_SET_FINGER_FORCE_LIMIT   = 0x47  # Set force limit of finger
HAND_CMD_SET_FINGER_POS_LIMIT     = 0x48  # Get current absolute position of finger
HAND_CMD_FINGER_START             = 0x49  # Start motor
HAND_CMD_FINGER_STOP              = 0x4A  # Stop motor
HAND_CMD_SET_FINGER_POS_ABS       = 0x4B  # Move finger to physical position, [0, 65535]
HAND_CMD_SET_FINGER_POS           = 0x4C  # Move finger to logical position, [0, 65535]
HAND_CMD_SET_FINGER_ANGLE         = 0x4D  # Set first joint angle of finger
HAND_CMD_SET_THUMB_ROOT_POS       = 0x4E  # Move thumb root to preset position, {0, 1, 2}
HAND_CMD_SET_FINGER_POS_ABS_ALL   = 0x4F  # Set current absolute position of all fingers
HAND_CMD_SET_FINGER_POS_ALL       = 0x50  # Set current logical position of all fingers
HAND_CMD_SET_FINGER_ANGLE_ALL     = 0x51  # Set first joint angle of all fingers

HAND_CMD_SET_CUSTOM               = 0x5F  # Custom set command

# Auxiliary SET commands
HAND_CMD_SET_SELF_TEST_LEVEL      = 0x60  # Set self-test level, level, 0: wait command, 1: semi self-test, 2: full self-test
HAND_CMD_SET_BEEP_SWITCH          = 0x61  # Set beep ON/OFF
HAND_CMD_BEEP                     = 0x62  # Beep for duration if beep switch is on
HAND_CMD_SET_BUTTON_PRESSED_CNT   = 0x63  # Set button press count, for ROH calibration only
HAND_CMD_START_INIT               = 0x64  # Start init in case of SELF_TEST_LEVEL=0


CMD_ERROR_MASK                    = (1<<7)  # bit mask for command error



def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


# OHand bus context
class OHandContext:
    def __init__(self, serial, timeout, address_master=0x01):
        """
        Initialize OHandContext.

        Parameters
        ----------
        serial : str
            Path to serial port
        timeout : int
            Timeout in in milliseconds
        address_master : int, optional
            Address of master device, default is 0x01
        """
        self.serial_port = serial
        self.address_master = address_master
        self.timeout = timeout
        self.is_whole_packet = False
        self.decode_state = WAIT_ON_HEADER_0
        self.packet_data = bytearray(MAX_PROTOCOL_DATA_SIZE + 5)  # Including node_id, own_id, command_id, byte_cnt, data[], lrc
        self.send_buf = bytearray(MAX_PROTOCOL_DATA_SIZE + 7)  # Including header0, header1, address, master address, cmd, nb_data, lrc
        self.byte_count = 0

        self.lock_ = threading.Lock()


    def acquire_lock(self):
        self.lock_.acquire()


    def release_lock(self):
        self.lock_.release()


    def calc_lrc(ctx, lrcBytes, lrcByteCount):
        """
        Calculate the LRC for a given sequence of bytes
        :param lrcBytes: sequence of bytes to calculate LRC over
        :param lrcByteCount: number of bytes in the sequence
        :return: calculated LRC value
        """
        lrc = 0
        for i in range(lrcByteCount):
            lrc ^= lrcBytes[i]
        return lrc


    def send_cmd(self, addr, cmd, data=None):
        """
        Send a command to the hand with the given id, wait for the response and store it in the same data buffer,
        and return the error code and the remote error.

        :param addr: address of the hand to send the command to
        :param cmd: command id
        :param data: data to send with command, if any
        :return: one of the HAND_RESP_{SUCCESS,INVALID_CONTEXT,DATA_SIZE_TOO_BIG,UNMATCHED_ADDR,UNMATCHED_CMD,HAND_ERROR,INVALID_OUT_BUFFER_SIZE} values
        """
        if self.serial_port is None:
            return HAND_RESP_INVALID_CONTEXT

        nb_data = 0

        if data is not None:
            nb_data = len(data)

            if nb_data >= MAX_PROTOCOL_DATA_SIZE:
                return HAND_RESP_DATA_SIZE_TOO_BIG

        self.send_buf[0] = 0x55
        self.send_buf[1] = 0xAA
        self.send_buf[2] = addr
        self.send_buf[3] = self.address_master
        self.send_buf[4] = cmd
        self.send_buf[5] = nb_data

        for i in range(nb_data):
            self.send_buf[6 + i] = data[i]

        lrc = self.calc_lrc(self.send_buf[2:6 + nb_data], 4 + nb_data)
        self.send_buf[nb_data + 6] = lrc

        self.is_whole_packet = False
        self.decode_state = WAIT_ON_HEADER_0  # Reset to initial state

        self.serial_port.write(self.send_buf[:nb_data + 7])

        return HAND_RESP_SUCCESS


    def get_response(self, addr, cmd, time_out, resp_bytes):
        """
        Send a command to the hand with the given id, wait for the response and store it in the same data buffer,
        and return the error code and the remote error.

        :param addr: address of the hand to send the command to
        :param cmd: command id
        :param time_out: time out in milliseconds
        :param resp_bytes: buffer to store the response data
        :return: one of the HAND_RESP_{SUCCESS,INVALID_CONTEXT,TIMEOUT,UNMATCHED_ADDR,UNMATCHED_CMD,HAND_ERROR,INVALID_OUT_BUFFER_SIZE} values and remote error
        """
        remote_err = None

        if self is None or self.serial_port is None:
            return HAND_RESP_INVALID_CONTEXT, remote_err

        wait_start = time.time()
        wait_timeout = wait_start + time_out / 1000

        while not self.is_whole_packet:
            time.sleep(0.001)

            # print(f"in_waiting: {self.serial_port.in_waiting}")

            while self.serial_port.in_waiting > 0:
                data_bytes = self.serial_port.read(self.serial_port.in_waiting)
                for ch in data_bytes:
                    # print(f"data: {hex(ch)}")
                    self.on_data(ch)
                if self.is_whole_packet: break

            if (not self.is_whole_packet) and (wait_timeout < time.time()):
                # print(f"wait time out: {wait_timeout}, now: {time.time()}")
                self.decode_state = WAIT_ON_HEADER_0
                return HAND_RESP_TIMEOUT, remote_err

        # Validate LRC
        lrc = self.calc_lrc(self.packet_data, self.packet_data[DATA_CNT_BYTE_NUM] + 4)
        if lrc != self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]]:
            self.is_whole_packet = False
            return ERR_PROTOCOL_WRONG_LRC, remote_err

        # Check if response is an error
        if (self.packet_data[CMD_ID_BYTE_NUM] & CMD_ERROR_MASK) != 0:
            remote_err = self.packet_data[DATA_START_BYTE_NUM]
            return HAND_RESP_HAND_ERROR, remote_err

        if self.packet_data[OWN_ID_BYTE_NUM] != addr and addr != 0xFF:  # Broadcast
            self.is_whole_packet = False
            return HAND_RESP_UNMATCHED_ADDR, remote_err

        if self.packet_data[CMD_ID_BYTE_NUM] != cmd:
            self.is_whole_packet = False
            return HAND_RESP_UNMATCHED_CMD, remote_err

        # Copy response data
        if resp_bytes is not None:
            packet_byte_count = self.packet_data[DATA_CNT_BYTE_NUM]
            resp_bytes.clear()
            resp_data = self.packet_data[DATA_START_BYTE_NUM : DATA_START_BYTE_NUM + packet_byte_count]
            for v in resp_data: resp_bytes.append(v)

        self.is_whole_packet = False
        return HAND_RESP_SUCCESS, None


    def on_data(self, data):
        """
        Called when a new byte is received from the serial port. This function implements
        a state machine to decode the packet. If a whole packet is received, is_whole_packet
        is set to 1 and the packet is stored in packet_data.

        Args:
            data (int): The newly received byte

        Returns:
            None
        """
        if self is None:
            return

        if self.is_whole_packet:
            return  # Old packet is not processed, ignore

        # State machine implementation
        if self.decode_state == WAIT_ON_HEADER_0:
            if data == 0x55:
                self.decode_state = WAIT_ON_HEADER_1

        elif self.decode_state == WAIT_ON_HEADER_1:
            self.decode_state = WAIT_ON_ADDRESSED_NODE_ID if data == 0xAA else WAIT_ON_HEADER_0

        elif self.decode_state == WAIT_ON_ADDRESSED_NODE_ID:
            self.packet_data[NODE_ID_BYTE_NUM] = data
            self.decode_state = WAIT_ON_OWN_NODE_ID

        elif self.decode_state == WAIT_ON_OWN_NODE_ID:
            self.packet_data[OWN_ID_BYTE_NUM] = data
            self.decode_state = WAIT_ON_COMMAND_ID

        elif self.decode_state == WAIT_ON_COMMAND_ID:
            self.packet_data[CMD_ID_BYTE_NUM] = data
            self.decode_state = WAIT_ON_BYTE_COUNT

        elif self.decode_state == WAIT_ON_BYTE_COUNT:
            self.packet_data[DATA_CNT_BYTE_NUM] = data
            self.byte_count = data

            if self.byte_count > MAX_PROTOCOL_DATA_SIZE:
                self.decode_state = WAIT_ON_HEADER_0
            elif self.byte_count > 0:
                self.decode_state = WAIT_ON_DATA
            else:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_DATA:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM] - self.byte_count] = data
            self.byte_count -= 1

            if self.byte_count == 0:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_LRC:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]] = data

            if self.packet_data[NODE_ID_BYTE_NUM] == self.address_master:
                self.is_whole_packet = True

            self.decode_state = WAIT_ON_HEADER_0

        else:
            self.decode_state = WAIT_ON_HEADER_0


class OHandProtocol:

    def __init__(self, ctx):
        """
        Initialize the OHandProtocol class.

        :param ctx: OHandContext object
        """
        self.ctx = ctx


    def get_protocol_version(self, hand_id):
        """
        Get the protocol version of the hand with the given id.

        :param hand_id: address of the hand to get the protocol version from
        :return: error code, remote error code, major version, minor version
        """
        major = None
        minor = None

        self.ctx.acquire_lock()

        err = self.ctx.send_cmd(hand_id, HAND_CMD_GET_PROTOCOL_VERSION)

        if err == HAND_RESP_SUCCESS:
            ver = bytearray()  # To hold the major version
            err, remote_err = self.ctx.get_response(hand_id, HAND_CMD_GET_PROTOCOL_VERSION, self.ctx.timeout, ver)

            if err == HAND_RESP_SUCCESS:
                minor = ver[0]
                major = ver[1]

        self.ctx.release_lock()

        return err, remote_err, major, minor


    def set_custom(self, hand_id, speed=None, position=None, angle=None, get_flag=0x00):
        """
        Send a custom command to the hand with the given id.

        :param hand_id: address of the hand to send the command to
        :param speed: list of motor speed (0 to 100)
        :param position: list of motor position (0 to 100)
        :param angle: list of motor angle (radian)
        :return: error code, remote error code, current motor position, current motor angle, current motor current, current motor force, current motor status
        """
        data_flag = get_flag
        data = bytearray(1)

        if speed is not None:
            data_flag |= SUB_CMD_SET_SPEED
            for i in range(len(speed)):
                value = int(speed[i])
                value = clamp(value, 0, 65535)
                data.append(value & 0xFF)
                data.append((value >> 8) & 0xFF)

        if position is not None:
            data_flag |= SUB_CMD_SET_POS
            for i in range(len(position)):
                value = int(position[i])
                value = clamp(value, 0, 65535)
                data.append(value & 0xFF)
                data.append((value >> 8) & 0xFF)

        if angle is not None:
            data_flag |= SUB_CMD_SET_ANGLE
            for i in range(len(angle)):
                value = int(angle[i] * 100)  # scale
                if value < 0:
                    value += 65536
                value = clamp(value, 0, 65535)
                data.append(value & 0xFF)
                data.append((value >> 8) & 0xFF)

        data[0] = data_flag

        self.ctx.acquire_lock()

        err = self.ctx.send_cmd(hand_id, HAND_CMD_SET_CUSTOM, data)

        if err == HAND_RESP_SUCCESS:
            err, remote_err = self.ctx.get_response(hand_id, HAND_CMD_SET_CUSTOM, self.ctx.timeout, data)

        self.ctx.release_lock()

        # Parse data
        position = None
        angle = None
        current = None
        force = None
        status = None

        motor_cnt = 0
        data_entry = 0
        data_len = len(data)
        # print(f"data_len: {data_len}")

        if (data_flag & SUB_CMD_GET_POS): data_entry += 2
        if (data_flag & SUB_CMD_GET_ANGLE): data_entry += 2
        if (data_flag & SUB_CMD_GET_CURRENT): data_entry += 2
        if (data_flag & SUB_CMD_GET_FORCE): data_entry += 2
        if (data_flag & SUB_CMD_GET_STATUS): data_entry += 1

        if data_entry != 0:
            motor_cnt = int(data_len / data_entry)

        if motor_cnt * data_entry != data_len:
            return HAND_RESP_DATA_INVALID, None

        # print(f"motor_cnt: {motor_cnt}")
        offset = 0

        if (data_flag & SUB_CMD_GET_POS):
            position = []
            for i in range(motor_cnt):
                value = data[offset] | (data[offset + 1] << 8)
                position.append(float(value))
                offset += 2

        if (data_flag & SUB_CMD_GET_ANGLE):
            angle = []
            for i in range(motor_cnt):
                value = data[offset] | (data[offset + 1] << 8)
                angle.append(float(value))
                offset += 2

        if (data_flag & SUB_CMD_GET_CURRENT):
            current = []
            for i in range(motor_cnt):
                value = data[offset] | (data[offset + 1] << 8)
                current.append(float(value))
                offset += 2

        if (data_flag & SUB_CMD_GET_FORCE):
            force = []
            for i in range(motor_cnt):
                value = data[offset] | (data[offset + 1] << 8)
                force.append(float(value))
                offset += 2

        if (data_flag & SUB_CMD_GET_STATUS):
            status = []
            for i in range(motor_cnt):
                value = data[offset]
                status.append(int(value))
                offset += 1

        return err, remote_err, position, angle, current, force, status
