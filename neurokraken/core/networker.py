import serial
import serial.tools.list_ports

from datetime import datetime
from core.print0 import print0
import time, platform, inspect

class Networker(object):
    def __init__(self, serial_key:str='COM0', 
                 archivist_mode:bool=True, serial_in_log:dict={}, run_controls=None,
                 verbose_connection:int=2, verbose_communication:bool=False, verbose_teensy_debug:bool=True):
        """print(ser.BAUDRATES) provides an incomplete list of valid baudrates that
           can be used for networking. read(size=x) would be blocking without a provided
           timeout. Similarly write_timeout = None makes write() blocking until all 
           bytes are written

        Args:
            serial_key (str, optional): Either the USB port of the connected device i.e. COM3,
                                        or a segment of the USB connection info, like a Vendor ID or 
                                        Serial Number that can be used to recognize a device.
            dense_mode (bool, optional): Whether to run in Direct mode or history mode. In direct mode the
                                         current sensor values are received and provided to the ordered_in_values
                                         at networker.read(). This mode is generally fast and direct but might
                                         miss values at milliseconds when the main loop is delayed. 
                                         In history mode the arduino side keeps track of yet uncommunicated values
                                         and sends a history with timestamps. Besides providing the current value
                                         to ordered_in_values, this mode adds all received history to the
                                         serial_in_log. History mode guarantees millisecond precision sensor data
                                         data but might spend more time to process histories. Defaults to True.
            serial_in_log (dict, optional): The dictionary to update sensor data into when using history mode
            run_controls (optional): A class containing a bool .active to define whether or not to currently log in history mode.
            Using connection description or ID information can be preferable to a com port, as it is not subject to
            changes between computers. You can use the list_serial() function to check whether a device connection has
            exposes suitable keys for identification. Defaults to 'COM0'.
            verbose_connection (int, optional): priority threshold (0 to 5) for printing connection-related information.
                                                At 0 only the highest priority events will be printed, at 5 all. Defaults to 2
            verbose_communication (bool, optional): print communicated data/bytes for debugging. Defaults to False
            verbose_teensy_debug (bool, optional): print teensy originating debug information. Defaults to True
        """
    
        self.connect(serial_key)
    
        print0.set_topic_threshold('teensy_debug', 6 if verbose_teensy_debug else 0)
        print0.set_topic_threshold('communication', 6 if verbose_communication else 0)
        print0.set_topic_threshold('connection', verbose_connection)

        # archivist mode logging functionality
        self.archivist_mode = archivist_mode
        self.serial_in_log = serial_in_log
        self.run_controls = run_controls
        if self.run_controls is None:
            class Run_Controls:
                def __init__(self):
                    self.active = True
            self.run_controls = Run_Controls()

    def connect(self, serial_key:str='COM0'):
        if 'COM' in serial_key:
            self.com = serial_key
        else:
            ports = serial.tools.list_ports.comports()
            device_ports = []
            for port, desc, hwid in sorted(ports):
                if serial_key in hwid:
                    device_ports.append(port)
            if len(device_ports) > 0:
                self.com = device_ports[0]
            else:
                print0(f'No serial port contains the provided identifier: "{serial_key}" in its connection information.',
                       priority=0, color='red', topic='connection')

        self.baudrate = 115_200
        self.timeout = 0.4
        self.write_timeout = None
        self.ser = serial.Serial(self.com, baudrate=self.baudrate, timeout=self.timeout, 
                                 write_timeout=self.write_timeout)

    def initialize_communication(self, num_bytes_out=3):
        """initialize the communication by sending the teensy its first command bytes"""
        for i in range(num_bytes_out):
            self.ser.write(b'\x00')

    def reconnect(self, keep_trying=True, device_name='teensy'):
        """reconnect the serial connection.

        Args:
            keep_trying (bool, optional): 
                Run as a blocking while True: loop until the reconnection attempt succeeds. 
                If False only try one reconnect. Defaults to True.
            device_name (str, optional): 
                The device/connection name to be used in log messages. Defaults to 'teensy'.
        Returns:
            bool:
                reconnection attempt success. True/False when keep_trying=False.
                If keep_trying=True the function will run forever until it is able to
                reconnect and return True.
        """
        while True:
            try:
                print0(f'attempting reconnect to {device_name}...', 
                       priority=1, color='red', topic='connection')
                time.sleep(0.05)
                self.ser.close()
                self.ser = serial.Serial(self.com, baudrate=self.baudrate, timeout=self.timeout, 
                                         write_timeout=self.write_timeout)
                print0(f'reconnect to {device_name} successful', 
                       priority=1, color='green', topic='connection')
                return True
            except serial.SerialException as e:
                if not keep_trying:
                    return False

    def read_teensy_data(self, ordered_in_values):
        """Read the new data from the teensy and update the provided ordered_in_values with it.
        If the teensy hasn't responded yet no values will be updated and the function will return 
        False. If the teensy has sent additional bytes with a debug message, it will be 
        returned in the 2nd argument"""

        try:
            if self.ser.in_waiting != 0:
                if not self.archivist_mode:
                    # direct mode
                    #The teensy has answered => read the first byte = length of the sent data
                    length_data_in = self.ser.read(1)
                    length_data_in = int.from_bytes(length_data_in, 'little')
                    #get the bytes array with the provided length
                    data_in = self.ser.read(length_data_in)
                    print0(f'length of data_in from teensy: {length_data_in}, data_in bytes: {data_in}',
                           priority=3, color='green', topic='communication')
                    
                    #update ordered_in_values with the current measurements
                    byte_position = 0
                    for data_point in ordered_in_values.values():
                        bytes = data_in[byte_position:byte_position+data_point['byte_length']]
                        signed = True if data_point['encoding'] == int else False
                        #recalculate the signed or unsigned int from little endian ordered bytes
                        value = int.from_bytes(bytes, 'little', signed=signed)
                        data_point['value'] = value
                        byte_position += data_point['byte_length']
                    # --- Data will be logged by the main loop ---
                else:
                    # archivist mode - read list of changes and timestamps since last communication
                    for sens_name, sens_data in ordered_in_values.items():
                        length_history = self.ser.read(2)
                        length_history = int.from_bytes(length_history, 'little', signed=False)

                        # print(sens_name, length_history)
                        log_times = self.ser.read(length_history * 4)
                        num_values = 1
                        if type(sens_data['value'])==list:
                            num_values = len(sens_data['value'])
                        log_values = self.ser.read(length_history * sens_data['byte_length'] * num_values)

                        if length_history != 0:
                            signed = True if sens_data['encoding'] == int else False
                            if num_values == 1:
                                value = log_values[-sens_data['byte_length']:]
                                value = int.from_bytes(value, 'little', signed=signed)
                                sens_data['value'] = value
                                # print(sens_data['value'])
                                # --- Log the data ---
                                if sens_data['logging'] and self.run_controls.active:
                                    log_entry = self.serial_in_log.setdefault(sens_name, [])
                                    for i in range(length_history):
                                        t = log_times[i*4:i*4+4]
                                        t = int.from_bytes(t, 'little', signed=False)
                                        value = log_values[i*sens_data['byte_length']:i*sens_data['byte_length']+sens_data['byte_length']]
                                        value = int.from_bytes(value, 'little', signed=signed)
                                        log_entry.append((t, value))
                            else:
                                # example encoding for 3 communicated timepoints at 2 values of byte_length 2:
                                # t0, t1, t2 -> t0_v0b0, t0_v0b1, t0_v1b0, t0_v1b1 -> 
                                #               t1_v0b0, t1_v0b1, t1_v1b0, t1_v1b1 -> t2_v0b0, t2_v0b1, t2_v1b0, t2_v1b1
                                if sens_data['logging']:
                                    log_entry = self.serial_in_log.setdefault(sens_name, [])
                                for i in range(length_history):
                                    t = log_times[i*4:i*4+4]
                                    t = int.from_bytes(t, 'little', signed=False)
                                    values_bytes = log_values[i*sens_data['byte_length']*num_values:i*sens_data['byte_length']*num_values+(sens_data['byte_length']*num_values)]
                                    values = []
                                    for v in range(num_values):
                                        value = values_bytes[v*sens_data['byte_length']:v*sens_data['byte_length']+sens_data['byte_length']]
                                        value = int.from_bytes(value, 'little', signed=signed)
                                        values.append(value)
                                    if sens_data['logging'] and self.run_controls.active:
                                        log_entry.append((t, values))
                                sens_data['value'] = values

                debug_in = None
                if self.ser.in_waiting != 0:
                    length_debug_in = int.from_bytes(self.ser.read(1), 'little')
                    print0(f'teensy sent {length_debug_in} bytes additional debug information:',
                           priority=3, color='green', topic='communication')
                    debug_in = self.ser.read(length_debug_in)
                    debug_in = debug_in.decode()
                    print0(debug_in, priority=3, color='yellow', topic='teensy_debug')

                if self.ser.in_waiting != 0:
                    print0(f'{self.ser.in_waiting} input bytes are still available after ' + \
                           'reading connection', priority=2, color='red', topic='connection')
                    self.ser.reset_input_buffer()
                    print0(f'reset the input buffer to {self.ser.in_waiting} available bytes',
                           priority=2, color='red', topic='connection')

                return True, debug_in
            return False, None
        except serial.SerialException as e:
            print0(f'reading timeout at {datetime.now()}', 
                   priority=1, color='red', topic='connection')
            print0(f'Please make sure to use a shielded USB 2.0 cable kept distant from higher ' + \
                   f'voltage wires!', priority=1, color='red', topic='connection')
            self.reconnect()
            #make sure the input_buffer is empty and return True to continue the handshacke communication
            self.ser.reset_input_buffer()
            return True, None


    def write_teensy_data(self, ordered_out_values):
        """write the provided ordered_out_values to the teensy to control its behavior.
        Boolean values will be sent as byte \x01 for True and \x00 for False.
        The function returns a boolean for success or failure of the writing"""

        bytes_out = b''
        for data_point in ordered_out_values.values():
            if(data_point['encoding'] == bool):
                byte_out = b'\x01' if data_point['value'] == True else b'\x00'
            elif(data_point['encoding'] == 'uint'):
                byte_out = data_point['value'].to_bytes(data_point['byte_length'], 'little', signed=False)
            bytes_out += byte_out
            if data_point['reset_after_send'] == True:
                data_point['value'] = data_point['default']

        print0(f'sending {len(bytes_out)} bytes to teensy: {bytes_out} '+\
               f'at {datetime.now()} ({self.ser.out_waiting} bytes out_waiting)',
               priority=3, color='green', topic='communication')

        try:
            self.ser.write(bytes_out)
            return True
        except serial.SerialTimeoutException as e:
            print0(f'Write timeout at {datetime.now()}', priority=1, color='red', topic='connection')
            print0(f'Please make sure to use a shielded USB 2.0 cable kept distant from higher voltage wires!',
                   priority=1, color='red', topic='connection')
            self.reconnect()
            self.ser.write(bytes_out)
            return False

    def close(self):
        self.ser.close()

class Dummy_Networker():
    """A simple Dummy networker that instead of communicating with a teensy allows using local inputs like the keyboard or an agent.

    Time source depends on virtual_time_step_ms: when None, t_ms is read from the wall clock on each
    read_teensy_data() call (used by keyboard mode and visual agent mode). When set, t_ms is advanced
    deterministically by that delta per call instead (used by headless agent mode); the agent.act()
    callback is also skipped in this branch because the headless caller injects actions via Neurokraken.step()."""

    def __init__(self, mode='keyboard', agent=None, virtual_time_step_ms=None, *args, **kwargs):
        """
        Args:
            mode (str, optional): 'keyboard' to poll keys for serial_in values, or 'agent' to call agent.act()
                                  on each read_teensy_data (visual agent mode). Defaults to 'keyboard'.
            agent (object, optional): Object with an act() method and act_freq (Hz) attribute. Required for
                                      visual agent mode; ignored when virtual_time_step_ms is set (headless).
            virtual_time_step_ms (float, optional): When set, switches to deterministic virtual time: each
                                                    read_teensy_data() advances t_ms by this many ms and the
                                                    agent.act() callback is skipped. Defaults to None (wall-clock).
        """
        print('running dummy networker for keyboard inputs - no connected teensy needed.\n' +
              'Press ctrl+alt+k to toggle the key input recognition active/inactive')
        self.controlled_serial_in:list[str] = []
        self.start_time = None
        self.archivist_mode = False
        self.mode=mode
        self.agent = agent
        self.t_last_agent_act = 0
        # When set, time advances deterministically by this delta per read_teensy_data() call
        # instead of from the wall clock. Used by headless RL training so a tick is one tick
        # regardless of how long Python actually took.
        self.virtual_time_step_ms = virtual_time_step_ms

    def initialize_communication(self, num_bytes_out=3):
        pass

    def read_teensy_data(self, serial_in):
        if self.virtual_time_step_ms is not None:
            # headless / virtual time mode: advance the simulated clock by a fixed step
            # and skip wall-clock + agent.act() (actions are injected via Neurokraken.step())
            if 't_ms' in serial_in:
                serial_in['t_ms']['value'] += self.virtual_time_step_ms
            if 't_us' in serial_in:
                serial_in['t_us']['value'] += self.virtual_time_step_ms * 1000.0
            return True, None

        # time
        if self.start_time is None:
            self.start_time = time.time_ns() / 1_000_000.
        if 't_ms' in serial_in.keys():
            serial_in['t_ms']['value'] = (time.time_ns() / 1_000_000.) - self.start_time
        if 't_us' in serial_in.keys():
            serial_in['t_us']['value'] = (time.time_ns() / 1_000.) - self.start_time

        if self.mode == 'keyboard':
            # custom key presses
            if platform.system() == 'Windows':
                import keyboard

            if len(self.controlled_serial_in) == 0:
                self.controlled_serial_in = [k for k, v in serial_in.items() if 'keys' in v]

            for device in self.controlled_serial_in:
                keys_pressed = [keyboard.is_pressed(key) for key in serial_in[device]['keys']]
                num_args = len(inspect.signature(serial_in[device]['keys_control']).parameters)
                if num_args == 1:
                    serial_in[device]['value'] = serial_in[device]['keys_control'](keys_pressed)
                elif num_args == 2:
                    serial_in[device]['value'] = serial_in[device]['keys_control'](keys_pressed, serial_in[device]['value'])
        elif self.mode == 'agent':
            if serial_in['t_ms']['value'] > self.t_last_agent_act + (1000 / self.agent.act_freq):
                self.t_last_agent_act = serial_in['t_ms']['value']
                self.agent.act()

        return True, None

    def write_teensy_data(self, serial_out):
        for key, data_point in serial_out.items():
            if key == 'start_stop' and data_point['value'] == 1:
                self.start_time = time.time_ns() / 1_000_000.
            if data_point['reset_after_send'] == True:
                data_point['value'] = data_point['default']
        return True
    
    def close(self):
        pass

def list_serial():
    """List information of connected serial devices.
    """
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(port, desc, hwid)