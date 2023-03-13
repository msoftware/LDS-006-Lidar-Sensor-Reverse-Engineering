#!/usr/bin/env python3
import serial
from threading import Thread, Event
from enum import Enum
import numpy as np

class LDSSerialManager(object):
    

    # Start lidar sensor and read data
    def start(self):
        if self.__state == self.State.STOPPED:
            self.__start_condition.set()

    # Keep sensor spinning but dont read data
    def pause(self):
        self.__state = self.State.STOPPED

    # Stop sensor from spinning
    def stop(self):
        self.__has_received__data.clear()
        self.__state = self.State.STOPPED
        self.__stop_lds()

    # Return data from internal array
    def getItem(self, index=slice(0,359)):
        self.__has_received__data.wait()
        index = self.__correct_index(index)
        return self.__data[index]

    # Return data from internal array in bracket-representation
    def __getitem__(self, index):
        self.__has_received__data.wait()
        index = self.__correct_index(index)
        return self.__data[index]

    # Return data via callback function
    def registerCB(self, cb, index=range(360)):
        if cb:
            index = self.__correct_index(index)
            self.__cb_hash += 1
            self.__cb[self.__cb_hash] = [cb, index]
            return self.__cb_hash
        return None

    def unregisterCB(self, cb_hash):
        del self.__cb[cb_hash]

    # Set some state machine
    class State(Enum):
        STOPPED = 1
        STARTING = 2
        RUNNING = 3
        EXIT = 4

    # Initialise class
    def __init__(self, port, baudrate=115200 , timeout=5, filter_array_size=8):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            print("Could not open serial port!")
            raise e
        except Exception as e:
            print("Unknown error occured!")
            raise e
        self.__min_reflectivity = 10
        self.__state = self.State.STOPPED
        self.__start_condition = Event()
        self.__has_received__data = Event()
        self.__has_new__data = Event()
        self.__data = [0] * 360
        self.__reader_thread = Thread(target=self.__lds_reader, daemon=True)
        self.__reader_thread.start()
        self.__cb_hash = 0
        self.__cb = dict()
        self.__cb_thread = Thread(target=self.__cb_publisher, daemon=True)
        self.__cb_thread.start()
        self.NUM_OF_ENTRIES = 360
        self.min_deviation = 0.02
        self.max_deviation = 0.05
        self.__filter_array_index = 0
        self.__filter_array_size = filter_array_size
        filter_array_list = []
        for i in range(self.__filter_array_size):
            filter_array_list.append([20000]*self.NUM_OF_ENTRIES)
        self.__filter_array = np.array(filter_array_list, np.float32)
    def __enter__(self):
        return self
    
    # Shutdown class
    def __del__(self):
        self.close()
    def __exit__(self, ext_type, exc_value, traceback):
        self.close()
    def close(self):
        self.__state = self.State.EXIT
        self.__start_condition.set()
        self.__reader_thread.join()
        self.__stop_lds()
        self.ser.close()
        
    # Start lidar sensor
    def __start_lds(self):
        if self.ser.is_open:
            self.ser.write(b'$startlds$')
    
    # Stop lidar sensor from spinning
    def __stop_lds(self):
        if self.ser.is_open:
            self.ser.write(b'$stoplds$')

    def __correct_index(self, index):
        if isinstance(index, slice):
            index = slice(min(abs(index.start), self.NUM_OF_ENTRIES - 1) , min(index.stop, self.NUM_OF_ENTRIES - 1), index.step)
        elif isinstance(index, range):
            index = slice(min(abs(index.start), self.NUM_OF_ENTRIES - 1) , min(index.stop, self.NUM_OF_ENTRIES), index.step)
        else: # its a number
            if index < 0:
                index = 0
            elif index >= self.NUM_OF_ENTRIES:
                index = self.NUM_OF_ENTRIES - 1
        return index

    def __cb_publisher(self):
        while True:
            self.__has_new__data.wait()
            self.__has_new__data.clear()
            #self.__data2 = self.__filter_value(self.__data)
            for cb in self.__cb:
                self.__cb[cb][0](self.__data[self.__cb[cb][1]])

    def __lds_reader(self):
        try:
            __data_counter = 0
            while not self.__state == self.State.EXIT:
                if self.__state == self.State.STOPPED:
                    self.__start_condition.wait()
                    if not self.__state == self.State.EXIT:
                        self.__state = self.State.STARTING
                        self.__start_condition.clear()
                        self.__start_lds()
            
                elif self.__state == self.State.STARTING:
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
                        if self.__update_lidar__data(data):
                            __data_counter += 2 # it will read 4 values at once, yet we might give it some extra time
                        if __data_counter >= 360:
                            self.__has_received__data.set()
                            self.__state = self.State.RUNNING

                elif self.__state == self.State.RUNNING:
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
                        if self.__update_lidar__data(data):
                            __data_counter += 4
                            if __data_counter >= 360:
                                __data_counter = 0
                                self.__has_new__data.set()
                        else:
                            __data_counter = 0

        except serial.SerialException as e:
            print("Serial error occured!")
            raise e
        except Exception as e:
            print("Undefined error occured!")
            raise e

    def __get_int(self, lb, hb):
        return lb | (hb << 8)

    def __update_lidar__data(self, data):
        checksum2 = 0
        for x in range(20):
            checksum2 = checksum2 + data[x]
        checksum1 = self.__get_int (data[20], data[21])
        angle = (data[1] - 0xA0) * 4
        if checksum1 != checksum2 and angle >= 360:
            return False
        else:
            """
            distance = [0] * 4
            reflectivity = [0] * 4
            speed = get_int (data[2],data[3])
            for x in range(4):
                distance[x] =  get_int (data[4+x],data[5+x])
                reflectivity[x] = get_int (data[6+x],data[7+x])
            for x in range(4):
                if reflectivity[x] > min_reflectivity:
                    self.__data[(angle+x)% 360] = distance[x]
            """
            for x in range(4):
                if self.__get_int(data[6+4*x],data[7+4*x]) > self.__min_reflectivity:
                    self.__data[(angle+x)% self.NUM_OF_ENTRIES] = min(self.__get_int(data[4+4*x],data[5+4*x]), 20000)
                else:
                    self.__data[(angle+x)% self.NUM_OF_ENTRIES] = self.__data[(angle + x + self.NUM_OF_ENTRIES - 1)% self.NUM_OF_ENTRIES]
            return True
    
    """def __filter_value(self, data):
        curr_val = np.array(data, np.float32)
        deviation = np.array(np.abs(curr_val / self.__filter_array[self.__filter_array_index] - 1), np.float32)
        self.__filter_array_index = (self.__filter_array_index + 1) % self.__filter_array_size
        for i in range(deviation.size):
            if deviation[i] < self.min_deviation:
                curr_val[i] = self.__filter_array[self.__filter_array_index][i]
            elif deviation[i] > self.max_deviation:
                self.__filter_array[self.__filter_array_index][i] = curr_val[i]
                mean = 0
                for j in range(self.__filter_array_size):
                    mean += self.__filter_array[j][i]
                curr_val[i] = mean / self.__filter_array_size
            else:
                self.__filter_array[self.__filter_array_index][i] = curr_val[i]
        return curr_val.tolist()"""
    
    # Median filter for the data
    def __filter_value(self, data):
        curr_val = np.array(data, np.float32)
        self.__filter_array[self.__filter_array_index] = curr_val
        self.__filter_array_index = (self.__filter_array_index + 1) % self.__filter_array_size
        return np.median(self.__filter_array, axis=0).tolist()
        