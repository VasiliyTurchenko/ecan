#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 27 20:28:51 2024

@author: tvv
"""

import sys
from time import sleep, time
from ecan import Ecan, CAN_Packet_Filter, CAN_Message_Composer, dumper, dumper_green


def dumper_0x18feee00(msg):
    """
    65262 (0x18FEEE00) Engine Temperature #1: ET1 TX 1s
    MASTER Engine Coolant Temperature 110 1 1°C /bit, -40°C -40 to 210°C byte 1
    Engine Fuel Temperature 174 2 1°C /bit, -40°C -40 to 210°C byte 2
    Engine Oil Temperature 1 175 3,4 0,03125 /bit, -273°C -273 to 1735°C byte 3, 4
    Engine Intercooler Temperature 52 7 1°C /bit, -40°C -40 to 210°C byte 7
    """
    payload = msg[2]
    if (len(payload) != 8):
        print(f"dumper_0x18feee00 ERROR 1 : {len(payload)} != 8")
        return
    
    print(f"Engine Coolant Temperature (C) = {payload[0] - 40}")
    print(f"Engine Fuel Temperature (C) = {payload[1] - 40}")
    eoil = ((payload[2] + payload[3]*256)*0.03125) - 273
    print(f"Engine Oil Temperature (C) = {eoil}")
    print(f"Engine Intercooler Temperature (C) = {payload[6] - 40}")
    return


def dumper_0x18fef700(msg):
    """
    65271 (0x18FEF700) Vehicle Electrical Power: VEP TX 1s
    MASTER Electrical Potential 168 5,6 0.05V /bit, 0V 0 to 3212.75V
    """
    payload = msg[2]
    
    if (len(payload) != 8):
        print(f"dumper_0x18fef700 ERROR 1 : {len(payload)} != 8")
        return
    epot = ((payload[4] + payload[5]*256)*0.05)
    print(f"Electrical Potential (V) = {epot}")

def dumper_0x18fef500(msg):
    """
    Ambient Conditions: AMB TX 1s
    MASTER Barometric Pressure 108 1 0.5kPa /bit, 0kPa 0 to 125kPa
    Air Inlet Temperature 172 6 1°C /bit, -40°C -40 to 210°C    
    """
    payload = msg[2]
    if (len(payload) != 8):
        print(f"dumper_0x18fef500 ERROR 1 : {len(payload)} != 8")
        return
    print(f"Barometric Pressure (kPa) = {payload[0]*0.5}")
    print(f"Air Inlet Temperature = {payload[5]}")
    

def main():
    """The main() test executable"""
    ecan = Ecan(sys.argv[2])
    ecan.logger.info("Hello from main()")
    ecan.connect()
    ecan.open_channel(1, "250")

    ecan.channel1.add_filter(CAN_Packet_Filter(msg_id=0x18feee00, cb=dumper_0x18feee00))
    ecan.channel1.add_filter(CAN_Packet_Filter(msg_id=0x18fef700, cb=dumper_0x18fef700))
    ecan.channel1.add_filter(CAN_Packet_Filter(msg_id=0x18fef500, cb=dumper_0x18fef500))

    print(f"channel 1 num. of filters: {len(ecan.channel1.filters)}")
   
    # dump non-channes stuff
    timeout = time() + 10
    while (time() <= timeout):
        resp = ecan.reader.pop(0)
        if (len(resp) == 0):
            sleep(0.5)
        else:
            print(f"Non-channel message: {resp}")

    print("CH1 IDs seen:")
    for id in ecan.channel1.all_ids:
        print(hex(id))
        
    ecan.close_channel(1)
    ecan.disconnect()
    ecan.logger.info("Bye from main()")
    del ecan    


###############################################################################
if __name__ == "__main__":
    sys.argv = ["__", "-d", "/dev/ttyACM0",
                "-1", "250", "-2", "250", "-c", "all"]

    main()

########################## E. O. F. ###########################################