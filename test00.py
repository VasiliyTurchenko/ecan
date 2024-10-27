#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 27 16:46:08 2024

@author: tvv
"""

import sys
from time import sleep, time
from ecan import Ecan, CAN_Packet_Filter, CAN_Message_Composer, dumper, dumper_green

def main():
    """The main() test executable"""
    ecan = Ecan(sys.argv[2])

    ecan.logger.info("Hello from main()")

    ecan.connect()

    ecan.open_channel(1, "250")
    ecan.open_channel(2, "250")

# channel2 sends to ch1

    ecan.channel2.add_filter(CAN_Packet_Filter(msg_id=0xe0, cb=dumper))
    ecan.channel1.add_filter(CAN_Packet_Filter(msg_id=0xe0, cb=dumper_green))

    msg_0xe0 = CAN_Message_Composer(ecan.channel2, msg_id=0xe0, ext = False)
    
    print(f"channel 1 num. of filters: {len(ecan.channel1.filters)}")
    print(f"channel 2 num. of filters: {len(ecan.channel2.filters)}")

    print (msg_0xe0.send_message(b'\x5A\x5B\x5C\x5D\x5E\x5F\x60\x61'))

    # dump non-channes stuff
    timeout = time() + 1
    while (time() <= timeout):

#        print (msg_0xe0.send_message(b'\x5A\x5B\x5C\x5D\x5E\x5F\x60\x62'))
#        print (msg_0xe0.send_message(b'\x5A\x5B\x5C\x5D\x5E\x5F\x60\x63'))
#        print (msg_0xe0.send_message(b'\x5A\x5B\x5C\x5D\x5E\x5F\x60\x64'))
        
        
        resp = ecan.reader.pop(0)
        if (len(resp) == 0):
            sleep(0.5)
        else:
            print(f"Non-channel message: {resp}")

    print("CH1 IDs seen:")
    for id in ecan.channel1.all_ids:
        print(hex(id))
        
    print("CH2 IDs seen:")
    for id in ecan.channel2.all_ids:
        print(hex(id))
    

    ecan.close_channel(1)
    ecan.close_channel(2)

    ecan.disconnect()

    ecan.logger.info("Bye from main()")

    del ecan    


###############################################################################
if __name__ == "__main__":
    sys.argv = ["__", "-d", "/dev/ttyACM0",
                "-1", "250", "-2", "250", "-c", "all"]

    main()

########################## E. O. F. ###########################################
    