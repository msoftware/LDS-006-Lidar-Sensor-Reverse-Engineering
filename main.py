#!/usr/bin/env python3
import time, sys
from lds006 import LDSSerialManager

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
        def cb(x):
            print(x)
        lds.registerCB(cb)

        while True:
            try:
                #print(lds.getItem(range(360)))
                
                #print(lds[0:360])

                print("another second passed")                
                time.sleep(1)
            except KeyboardInterrupt:
                break
