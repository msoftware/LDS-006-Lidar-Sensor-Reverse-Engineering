#!/usr/bin/env python3
import time, sys
from lds006.lds006 import LDSSerialManager

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
        while True:
            try:
                # You may use integers, slices, range
                # via []-Operator or getItem()
                # by default all 360 entries are returned
                #print(lds[1])
                #print(lds[0:360])
                #print(lds.getItem(range(360)))

                print(lds.getItem())

                time.sleep(1)
            except KeyboardInterrupt:
                break
