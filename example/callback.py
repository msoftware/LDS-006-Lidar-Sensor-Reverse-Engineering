#!/usr/bin/env python3
import time, sys
from lds006.lds006 import LDSSerialManager

def cb(x):
    print(x)

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
        lds.registerCB(cb, 0)

        while True:
            try:
                print("another second passed")                
                time.sleep(1)
            except KeyboardInterrupt:
                break
