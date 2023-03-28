#!/usr/bin/env python3
import time, sys
from lds006.lds006 import LDSSerialManager
import lds006.msgLDS_pb2 as msgLDS_pb2

def cb(x):
    msg = msgLDS_pb2.msgLDS()
    msg.ParseFromString(x)
    print("Length of data: " + str(len(x)) + "bytes")
    print("Measurement point 'msg.data[0]': {\n" + str(msg.data[0]) + "\n}")
    print("Mean of good data: " + str(msg.mean))
    print("Standard deviaton of good data: " + str(msg.pstdev))

if __name__ == "__main__":

    # Fallback for broken library
    # usage: python main.py /dev/serial0 stop
    if len(sys.argv) == 3 and sys.argv[2] == "stop":
        lds = LDSSerialManager(sys.argv[1])
        lds.stop()
        sys.exit(0)
    
    # Default program
    # usage: python main.py /dev/serial0
    print("Connecting to: " + sys.argv[1])
    with LDSSerialManager(sys.argv[1]) as lds:
        lds.start()
        # Callback is registered and executed every time new data is available. Prints only 1 byte for readability
        lds.registerCB(cb)

        while True:
            try:
                print("another second passed")                
                time.sleep(1)
            except KeyboardInterrupt:
                break
