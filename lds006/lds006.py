#!/usr/bin/env python3
import serial
from threading import Thread, Event
from enum import Enum

class LDSSerialManager(object):
    
    # Set some state machine
    class State(Enum):
        STOPPED = 1
        STARTING = 2
        RUNNING = 3
        EXIT = 4

    # Initialise code
    def __init__(self, port):
        self._class_startup(port)
    def __enter__(self):
        return self
    def _class_startup(self, port, baudrate=115200 , timeout=5):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            print("Could not open serial port!")
            raise e
        except Exception as e:
            print("Unknown error occured!")
            raise e
        self._min_reflectivity = 10
        self._state = self.State.STOPPED
        self._start_condition = Event()
        self._has_received_data = Event()
        self._has_new_data = Event()
        self._data = [0] * 360
        self._reader_thread = Thread(target=self._lds_reader, daemon=True)
        self._reader_thread.start()
        self._cb = []
        self._cb_thread = Thread(target=self._cb_publisher, daemon=True)
        self._cb_thread.start()
        self.NUM_OF_ENTRIES = 360
    
    # Shutdown code
    def __del__(self):
        self.close()
    def __exit__(self, ext_type, exc_value, traceback):
        self.close()
    def close(self):
        self._state = self.State.EXIT
        self._start_condition.set()
        self._reader_thread.join()
        self._stop_lds()
        self.ser.close()
        
    # Start lidar sensor
    def _start_lds(self):
        if self.ser.is_open:
            self.ser.write(b'$startlds$')

    # Stop lidar sensor from spinning
    def _stop_lds(self):
        if self.ser.is_open:
            self.ser.write(b'$stoplds$')

    # Start lidar sensor and read data
    def start(self):
        if self._state == self.State.STOPPED:
            self._start_condition.set()

    # Keep sensor spinning but dont read data
    def pause(self):
        self._state = self.State.STOPPED

    # Stop sensor from spinning
    def stop(self):
        self._has_received_data.clear()
        self._state = self.State.STOPPED
        self._stop_lds()

    def _correct_index(self, index):
        if isinstance(index, slice) or isinstance(index, range):
            index = slice(min(abs(index.start), self.NUM_OF_ENTRIES - 1) , min(index.stop, self.NUM_OF_ENTRIES - 1), index.step)
        else: # its a number
            if index < 0:
                index = 0
            elif index >= self.NUM_OF_ENTRIES:
                index = self.NUM_OF_ENTRIES - 1
        return index

    # Return data from internal array
    def getItem(self, index=slice(0,359)):
        self._has_received_data.wait()
        index = self._correct_index(index)
        return self._data[index]

    # Return data from internal array in bracket-representation
    def __getitem__(self, index):
        self._has_received_data.wait()
        index = self._correct_index(index)
        return self._data[index]

    # Return data via callback function
    def registerCB(self, cb, index=slice(0,359)):
        if cb:
            index = self._correct_index(index)
            self._cb.append([cb, index])

    def _cb_publisher(self):
        while True:
            self._has_new_data.wait()
            self._has_new_data.clear()
            for cb in self._cb:
                cb[0](self._data[cb[1]])

    def _lds_reader(self):
        try:
            _data_counter = 0
            while not self._state == self.State.EXIT:
                if self._state == self.State.STOPPED:
                    self._start_condition.wait()
                    if not self._state == self.State.EXIT:
                        self._state = self.State.STARTING
                        self._start_condition.clear()
                        self._start_lds()
            
                elif self._state == self.State.STARTING:
                    self.ser.reset_input_buffer()
                    start = self.ser.read(1)
                    while not ((start[0] == 0xFA) or (start[0] == 0x5A)):  
                        start = self.ser.read(1)
                    start = start + self.ser.read(1)
                    if start[0] == 0x5A and start[1] == 0xA5:
                        # Initial data package has 4 byte, we just ignore that one
                        self.ser.read(2)
                    elif start[0] == 0xFA and start[1] >= 0xA0 and start[1] != 0xFB:
                        # Subsequent data packages have 22 bytes, fill data buffer for the first time
                        data = start + self.ser.read(20)
                        if self._update_lidar_data(data):
                            _data_counter += 2 # it will read 4 values at once, yet we might give it some extra time
                        if _data_counter >= 360:
                            self._has_received_data.set()
                            self._state = self.State.RUNNING

                elif self._state == self.State.RUNNING:
                    data = self.ser.read(1)
                    while data[0] != 0xFA:
                        # Wait for start byte
                        data = self.ser.read(1)
                    data = data + self.ser.read(1)
                    if data[1] == 0xFB or data[1] < 0xA0:
                        # Check that next byte is valid speed
                        pass
                    else:
                        data = data + self.ser.read(20)
                        if self._update_lidar_data(data):
                            _data_counter += 4
                            if _data_counter >= 360:
                                _data_counter = 0
                                self._has_new_data.set()


        except serial.SerialException as e:
            print("Serial error occured!")
            raise e
        except Exception as e:
            print("Undefined error occured!")
            raise e

    def _get_int(self, lb, hb):
        return lb | (hb << 8)

    def _update_lidar_data(self, data):
        checksum2 = 0
        for x in range(20):
            checksum2 = checksum2 + data[x]
        checksum1 = self._get_int (data[20], data[21])
        if checksum1 != checksum2:
            return False
        else:
            angle = (data[1] - 0xA0) * 4
            """
            distance = [0] * 4
            reflectivity = [0] * 4
            speed = get_int (data[2],data[3])
            for x in range(4):
                distance[x] =  get_int (data[4+x],data[5+x])
                reflectivity[x] = get_int (data[6+x],data[7+x])
            for x in range(4):
                if reflectivity[x] > min_reflectivity:
                    self._data[(angle+x)% 360] = distance[x]
            """
            for x in range(4):
                if self._get_int(data[6+x],data[7+x]) > self._min_reflectivity:
                    self._data[(angle+x)% 360] = self._get_int(data[4+x],data[5+x])
            return True
