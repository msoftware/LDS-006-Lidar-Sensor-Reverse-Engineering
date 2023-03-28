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
                # You may use integers
                # via []-Operator or getItem()
                print("Data at position 1:" )
                print(" Using brackets 'lds[1]': " + str(lds[1]))
                print(" Using function 'lds.getItem(1)': " + str(lds.getItem(1)))
                print("Number of data points: " + str(len(lds)))
                actual_len = 0
                for i in lds:
                    actual_len+=1
                print("Number of good data: " + str(actual_len))
                time.sleep(1)
            except KeyboardInterrupt:
                break
