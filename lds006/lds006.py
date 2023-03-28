#!/usr/bin/env python3
import serial
from threading import Thread, Event, Lock
from enum import Enum
from statistics import pstdev
import msgLDS_pb2

class LDSSerialManager(object):  

    # Start lidar sensor and read data
    def start(self):
        if self.__state == self.State.STOPPED:
            if not self.__reader_thread.is_alive():
                self.__reader_thread.start()
            if not self.__cb_thread.is_alive():
                self.__cb_thread.start()
            self.__start_condition.set()

    # Keep sensor spinning but dont read data
    def pause(self):
        self.__state = self.State.STOPPED

    # Stop sensor from spinning
    def stop(self):
        self.__has_received_data.clear()
        self.__state = self.State.STOPPED
        self.__stop_lds()

    # Return data from internal array
    def getItem(self, index: int):
        self.__has_received_data.wait()
        index = self.__correct_index(index)
        with self.__data_lock:           
            match = None
            low_next = self.__data_backend.data[0]
            high_next = self.__data_backend.data[0]
            for i in self.__data_backend.data:
                if i.angle == index:
                    match = i
                    break
                if i.angle > low_next.angle and low_next.angle < index:
                    low_next = i
                elif i.angle < high_next.angle and high_next.angle > index:
                    high_next = i
            if match:
                return match.distance
            else:
                return (low_next.distance + high_next.distance) / 2.
    # Return data from internal array in bracket-representation
    def __getitem__(self, index: int):
        return self.getItem(index)
    # Return all data from internal array
    def __iter__(self):
        data_cp = None
        with self.__data_lock:
            data_cp = self.__data_backend
        for i in data_cp.data:
            if self.__include_bad_data:
                yield i.distance
            else:
                if i.certainty == True:
                    yield i.distance
    # Get length of data array
    def __len__(self):
        return len(self.__data_backend.data)

    # Return data via callback function
    def registerCB(self, cb):
        if cb:
            self.__cb_hash += 1
            self.__cb[self.__cb_hash] = cb
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
    # extend message object for statistics
    class __msgLDS_wrapper():
        def __init__(self, msgLDS_pb2_msgLDS):
            self.__pb_msg = msgLDS_pb2_msgLDS
        def __len__(self):
            return len(self.__pb_msg.data)
        def __iter__(self):
            for attr in self.__pb_msg.data:
                yield attr.distance

    # Initialise class
    def __init__(self, port, baudrate=115200 , timeout=5, min_reflectivity=10, include_bad_data=False):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            print("Could not open serial port!")
            raise e
        except Exception as e:
            print("Unknown error occured!")
            raise e
        self.__min_reflectivity = min_reflectivity
        self.__include_bad_data = include_bad_data
        self.NUM_OF_ENTRIES = 360
        self.__state = self.State.STOPPED
        self.__start_condition = Event()
        self.__has_received_data = Event()
        self.__has_updated_data = Event()
        self.__reader_thread = Thread(target=self.__lds_reader, daemon=True)
        self.__cb_hash = 0
        self.__cb = dict()
        self.__cb_thread = Thread(target=self.__cb_publisher, daemon=True)      
        self.__data_append = msgLDS_pb2.msgLDS()
        self.__data_backend = msgLDS_pb2.msgLDS()
        self.__bad_data = msgLDS_pb2.msgLDS()
        self.__data_lock = Lock() # prevents data_backend from being deleted while being read
        self.__resetDeviation()

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
        #if self.__reader_thread.is_alive():
        #    self.__reader_thread.join()
        #if self.__cb_thread.is_alive():
        #    self.__cb_thread.join()
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
        if(type(index) == int):
            if index < 0:
                index = 0
            elif index >= self.NUM_OF_ENTRIES:
                index = self.NUM_OF_ENTRIES - 1
            return index
        else:
            raise TypeError("Expecting index to be of type: " + type(int))

    def __cb_publisher(self):
        while True:
            self.__has_received_data.wait()
            self.__has_updated_data.wait()
            self.__has_updated_data.clear()
            with self.__data_lock:
                data_stream = self.__data_backend.SerializeToString()
            for cb in self.__cb:
                self.__cb[cb](data_stream)

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
                        # Initial data package has 4 byte, just ignore that one
                        self.ser.read(2)
                    elif start[0] == 0xFA and start[1] >= 0xA0 and start[1] != 0xFB:
                        # Subsequent data packages have 22 bytes, fill data buffer for the first time
                        data = start + self.ser.read(20)
                        if self.__update_lidar__data(data):
                            __data_counter += 2 # it will read 4 values at once, yet we might give it some extra time
                            if __data_counter >= 360:
                                self.__has_received_data.set()
                                self.__state = self.State.RUNNING

                elif self.__state == self.State.RUNNING:
                    data = self.ser.read(1)
                    while data[0] != 0xFA:
                        # Wait for start byte
                        data = self.ser.read(1)
                    data = data + self.ser.read(1)
                    if data[1] == 0xFB or data[1] < 0xA0:
                        # Check that next byte is valid angle
                        pass
                    else:
                        data = data + self.ser.read(20)
                        self.__update_lidar__data(data)

        except serial.SerialException as e:
            print("Serial error occured!")
            raise e
        except Exception as e:
            print("Unexpected error occured!")
            raise e

    def __resetDeviation(self):
        self.__mean = 10000
        self.__mean_old = 10000
        self.__data_backend.mean = 10000
        self.__deviation = 30000
        self.__data_backend.pstdev = 30000


    def __get_int(self, lb, hb):
        return lb | (hb << 8)

    def __update_lidar__data(self, data):
        checksum2 = 0
        for x in range(20):
            checksum2 = checksum2 + data[x]
        checksum1 = self.__get_int (data[20], data[21])
        angle = (data[1] - 0xA0) * 4
        if angle == 0:
            # swap writing and reading storage, send data
            with self.__data_lock:
                self.__data_append, self.__data_backend = self.__data_backend, self.__data_append
                self.__data_append.Clear()
                t = self.__msgLDS_wrapper(self.__data_backend)
                if len(t) > 140:
                    self.__mean = self.__mean / len(t)
                    self.__data_backend.mean = self.__mean
                    self.__deviation = pstdev(t, self.__mean)
                    self.__data_backend.pstdev = self.__deviation
                else:
                    self.__resetDeviation()
                if not self.__include_bad_data:
                    # for convenience send all data 
                    self.__data_backend.data.extend(self.__bad_data.data)
                    self.__bad_data.Clear()
            self.__mean_old = self.__mean
            self.__mean = 0
            self.__has_updated_data.set()
        if checksum1 != checksum2:
            return False               
        for x in range(4):
            msg = msgLDS_pb2.msgLDS.containerLDS()
            msg.angle = angle + x
            msg.distance = self.__get_int(data[4+4*x],data[5+4*x])
            msg.certainty = False
            good_expression = (self.__get_int(data[6+4*x],data[7+4*x]) > self.__min_reflectivity and abs(msg.distance - self.__mean_old) < 2 * self.__deviation)
            if self.__include_bad_data:
                self.__mean += _msg.distance
                if good_expression:
                    msg.certainty = True
                self.__data_append.data.append(msg)
            else:
                if good_expression:
                    msg.certainty = True
                    self.__mean += msg.distance
                    self.__data_append.data.append(msg)
                else:
                    self.__bad_data.data.append(msg)
        return True
        