#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  9 00:16:20 2024

@author: tvv
"""

import serial
import sys
import logging
import collections
# from multiprocessing import Process, Lock
from threading import Thread, Lock
from time import sleep, time
from datetime import datetime
from colorama import Fore, Back, Style


cmds = {
    "cmd_connect_ecan": b"\xf0\x01\x0d\x0a",
    "cmd_disconnect_ecan": b"\xf0\x0f\x0d\x0a",
}

PACKET_LEN = 20


def init_logger():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter(
        '[%(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s'))
    logger.addHandler(console_handler)
    return logger


def dev_null(m):
    """ Default packet sink, does nothing"""
    return


def dumper(m):
    """Just a dumper"""
    print(f"MSG ID:{hex(m[0])}  EXT:{m[1]}  DATA:{m[2].hex(sep=' ')}  TS:{m[3]}")
    return

def dumper_green(m):
    """Just a dumper"""
    print(Fore.GREEN + f"MSG ID:{hex(m[0])}  EXT:{m[1]}  DATA:{m[2].hex(sep=' ')}  TS:{hex(m[3])}")
    print(Style.RESET_ALL)
    return


def dumper_raw_hex(m, text = 'DUMP: '):
    """ Prints hex bytes as it is"""
    print(f"{text}{m.hex(sep=' ')}")
    return


def dumper2(m):
    print(f"MSG ID:{hex(m[0])} DATA:{m[2].hex(sep=' ')}")
    return
    

class CAN_Packet_Filter:
    """ filter by CAN packet id"""

    def __init__(self, msg_id=-1, cb=dev_null):
        self.msg_id = msg_id
        self.cb = cb

        self.command = 0x00
        self.chan = 0x00
        self.cmd = 0x00
        self.ext = False
        self.len = 0x00
        self.data = b''
        self.ts = 0x00

        """
        ff 04 01 08 [10 02 00 00] [00 00 00 00 00 ff ff ff] d5073a00 std 1
        ff 04 01 82 [91 0b 00 00] [02 03 00 00 00 00 00 00] 21073a00 ext 1
        ff 04 02 88 [91 90 10 14] [00 01 00 6e 91 00 00 00] 7f177b00 ext 2
        ff 04 02 08 [91 04 00 00] [ff 90 01 00 00 00 00 00] 50147b00 std 2
        ff 03 01 00 [00 00 00 00] [00 00 00 00 00 00 00 00] 00010000 empty 1
        ff 04 02 87 [91 90 10 14] [00 01 00 6a 00 00 00 00] 1fc1ef00 ext 2 7 bytes
        """

    def filter(self, raw_msg: bytearray):
        """ parses and processes the packet"""
        # 1.split packet by fields
        if (len(raw_msg) != PACKET_LEN) or (raw_msg[0] != 0xFF):
            return False

        if raw_msg[1] != 0x04:
            return False

        if (raw_msg[3] & 0x80) == 0x80:
            tmp_ext = True
        else:
            tmp_ext = False

        if tmp_ext:
            tmp_id = raw_msg[4] + (raw_msg[5] << 8) + (raw_msg[6] << 16) + (raw_msg[7] << 24)
        else:
            tmp_id = raw_msg[4] + (raw_msg[5] << 8)

        if tmp_id != self.msg_id:
            return False

        # our packet
        self.command = raw_msg[1]
        self.chan = raw_msg[2]
        if (raw_msg[3] & 0x80) == 0x80:
            self.ext = True
        else:
            self.ext = False
        self.len = raw_msg[3] & 0x0F
        self.data = bytearray(raw_msg[8: 8 + self.len])
        self.ts = raw_msg[16] + (raw_msg[17] << 8) + (raw_msg[18] << 16) + (raw_msg[19] << 24)
        if self.cb != None:
            self.cb([self.msg_id, self.ext, self.data, self.ts])
        return True


class CAN_Message_Composer:
    """ Makes a CAN message """
   
    def __init__(self, ecan_channel, msg_id=-1, ext = False):
        self.msg_id = msg_id
        self.ext = ext
        self.sent = False
        self.ecan_channel = ecan_channel
        self.raw_output = b''
        self.tpl = bytearray(b'\xf0\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
        self.tpl[2] = ecan_channel.chNr
        if self.ext:
            self.tpl[3] = self.tpl[3] | 0x80
        
        m = self.msg_id.to_bytes(4, byteorder = 'little')
        self.tpl[4] = m[0]
        self.tpl[5] = m[1]
        self.tpl[6] = m[2]
        self.tpl[7] = m[3]
        
        self.tsb = b''
    
    def send_message(self, data = b''):
        out_msg = self.tpl
        # f0 05 01 08 ff 07 00 00 ->44 55 44 66 44 77 44 88<- 78 46 23 01        
        # f0 05 01 01 ff 07 00 00 ->65 00 00 00 00 00 00 00<- 78 46 23 01 -->
        # ff 05 01 01 78 46 23 01 _a1 04 b5 02_ 00 00 00 00 00 00 00 00 <-- sent OK
        
        # f0 05 01 01 ff 07 00 00 65 00 00 00 00 00 00 00 78 46 23 01 -->
        # ff 05 01 00 78 46 23 01 00 00 00 00 00 00 00 00 00 00 00 00 <-- sent timepout
        
        dlen = len(data)
        if dlen > 8:
            raise Exception("Byte array is longer than 8 bytes!")
        out_msg[3] = out_msg[3] | dlen
        for i in range (0, dlen):
            out_msg[8 + i] = data[i]
        
        ts = int(time()*1000)
        tsb = ts.to_bytes(8, byteorder = 'little')
       
        out_msg[16] = tsb[0]
        out_msg[17] = tsb[1]
        out_msg[18] = tsb[2]
        out_msg[19] = tsb[3]
        
        # dumper_raw_hex(out_msg, text = "to send: ")
        self.raw_output = out_msg
        self.tsb = tsb
        retval = self.ecan_channel.send( (self.raw_output, tsb[0:4]) )
        return retval


class Ecan:
    """Class representing a Generic ECAN device"""

###################################################################################################
    class EcanChannel:
        """Class representing an ECAN's channel"""

        def __init__(self, nr: int, parent):
            self.parent = parent
            self.chNr = nr
            self.chOpen = False
            self.filters = list()    # packet filters list
            self.please_quit = False     # fot the filter thread
            self.all_ids = list()       # all the IDs met go here
            self.send_ack = b'\xff\x05' + self.chNr.to_bytes(1, byteorder = 'little')
            self.last_send_ack = b''
            self.send_mutex = Lock()
            
            self.ch1_status_msg = b"\xff\x03\x01"
            self.ch2_status_msg = b"\xff\x03\x02"
            
            self.ch_status_msg = b''
            if self.chNr == 1:
                self.ch_status_msg = self.ch1_status_msg
            elif self.chNr == 2:
                self.ch_status_msg = self.ch2_status_msg
            else:
                self.parent.logger.error("self.ch_status_msg error")
                
        def open(self, br: str = "0", custom=False, pre: int = 0, tbs1: int = 0, tbs2: int = 0):

            def cmd_open_channel(chan, br="0", custom=False, pre=0, tbs1=0, tbs2=0):
                """Returns a bytearray with the correc open command"""
                # template
                tpl = b"\xf0\x02\x55\x66\x66\x77\x88\x0d\x0a"
                fixed_brs = {"5":  b"\xd0\x02\x0f\x02",
                             "10":  b"\x68\x01\x0f\x02",
                             "20":  b"\xb4\x00\x0f\x02",
                             "50":  b"\x48\x00\x0f\x02",
                             "100": b"\x28\x00\x0d\x02",
                             "125": b"\x24\x00\x0b\x02",
                             "250": b"\x12\x00\x0b\x02",
                             "500": b"\x08\x00\x0c\x03",
                             "800": b"\x05\x00\x0c\x03",
                             "1000": b"\x04\x00\x0c\x03"}

                ret = bytearray(tpl)
                ret[2] = chan

                if not custom:
                    if br not in fixed_brs:
                        raise Exception("Baud rate value is incorrect")

                    pre_tb = bytearray(fixed_brs[br])
                    ret[3] = pre_tb[0]
                    ret[4] = pre_tb[1]
                    ret[5] = pre_tb[2]
                    ret[6] = pre_tb[3]

                else:
                    return tpl
                return ret

            req = cmd_open_channel(self.chNr, br, custom, pre, tbs1, tbs2)
            self.parent.logger.debug(f"open request: {req}")
            self.parent.writer.push(req)

            expected_resp = bytearray(req)
            expected_resp[0] = 255
            expected_resp[7] = 0
            expected_resp[8] = 0
            for i in range(9, PACKET_LEN):
                expected_resp.append(0)

            timeout = time() + 0.02
            while (time() <= timeout):
                resp = self.parent.reader.pop(self.chNr)
                if (len(resp) == 0):
                    sleep(0)
                else:
                    if (expected_resp == resp):
                        self.parent.logger.debug(f"open request responce ({len(resp)} bytes): {resp}")
                        self.chOpen = True
                        break

            if (not self.chOpen):
                self.parent.logger.error(f"failed to open channel {self.chNr}")
                self.parent.logger.error(f"expected = {expected_resp}")
            else:
                self.parent.logger.info(f"Channel {self.chNr} open success (br = {br} bps)")
                self.parent.logger.debug(f"Channel {self.chNr} open!")

                self.filter_thread = Thread(target=self.run_filters)
                self.filter_thread.start()
            return self.chOpen

        def close(self):
            if not self.chOpen:
                return

            self.please_quit = True
            self.filter_thread.join(0.1)

            if (self.chNr == 1):
                self.parent.writer.push(b"\xf0\x02\x01\x00\x00\x00\x00\x0d\x0a")
                exp_resp = b"\xff\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
            else:
                self.parent.writer.push(b"\xf0\x02\x02\x00\x00\x00\x00\x0d\x0a")
                exp_resp = b"\xff\x02\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"

            timeout = time() + 0.1
            while (time() <= timeout):
                resp = self.parent.reader.pop(self.chNr)
                if (len(resp) == 0):
                    sleep(0.01)
                else:
                    if (resp == exp_resp):
                        self.chOpen = False
                        break
            if not (self.chOpen):
                self.parent.logger.debug(f"Channel {self.chNr} closed!")
            else:
                self.parent.logger.error(f"Channel {self.chNr} IS NOT closed!")

        def add_filter(self, filter):
            """ adds filter to the filter chain"""
            not_found = True
            for f in self.filters:
                if f.msg_id == filter.msg_id:
                    not_found = False
                    break

            if not_found:
                self.filters.append(filter)
                self.parent.logger.info(f"The filter for msg_id = {hex(filter.msg_id)} added to the ch. {self.chNr}!")
                return True

            self.parent.logger.info(f"The filter for msg_id = {hex(filter.msg_id)} not added to the ch. {self.chNr}, already in the chain!")
            return False

        def collect_ids(self, raw_msg):
            """ collects new packets ids """
            if (len(raw_msg) != PACKET_LEN) or (raw_msg[0] != 0xFF):
                return

            if raw_msg[1] != 0x04:
                return

            if (raw_msg[3] & 0x80) == 0x80:
                tmp_ext = True
            else:
                tmp_ext = False

            if tmp_ext:
                tmp_id = raw_msg[4] + (raw_msg[5] << 8) + (raw_msg[6] << 16) + (raw_msg[7] << 24)
            else:
                tmp_id = raw_msg[4] + (raw_msg[5] << 8)
            
            if (tmp_id not in self.all_ids):
                self.all_ids.append(tmp_id)
            
        def send(self, d):
            retval = "OK"
            self.send_mutex.acquire()
            self.parent.writer.push(d[0])

            timeout = time() + 0.05
            while (time() <= timeout):
                if len(self.last_send_ack) == 0:
                    sleep(0)
                    continue
                break
            
            if len(self.last_send_ack) == 0:
                self.parent.logger.error("The send_ack was not received")
                retval = "NAK"
            elif len(self.last_send_ack) != 20:
                self.parent.logger.error(f"The send_ack is incorrect: {self.last_wr_ack}")
                retval = "INV"
            elif self.last_send_ack[4:8] != d[1]:
                self.parent.logger.error(f"The send_ack is from different transaction: {self.last_send_ack} vs. {d[1]}")
                retval = "DIFF"
            elif self.last_send_ack[3] == 0:
                self.parent.logger.error("The send() timeout")
                retval = "TOUT"
            elif self.last_send_ack[3] == 1:
                retval = "SENT"
            else:
                self.parent.logger.error(f"Unknown send_ack: {self.last_wr_ack}")
                retval = "UNK"
            self.last_send_ack = b''
            self.send_mutex.release()
            self.parent.logger.info(f"{retval}")
            return retval

        def proc_sm(self, msg):
#            self.parent.logger.debug(f"Ch.{self.chNr} - st.msg.: {msg.hex(sep=' ')}")
            return

        def run_filters(self):
            """ Run in separate thread """
            while not self.please_quit:
                msg = self.parent.reader.pop(self.chNr)
                if (len(msg) != 0):
                    if (msg[0:3] == self.ch_status_msg):
                        self.proc_sm(msg)
                        continue

                    if (msg[0:3] == self.send_ack):
                        self.last_send_ack = msg
#                        self.parent.logger.debug(f"Channel {self.chNr} - send_ack in {self.last_send_ack}!")
                        continue
                    
                    self.collect_ids(msg)
#                    self.parent.logger.debug(f"Channel {self.chNr}: pop msg: {msg}")
                    filter_done = False
                    for f in self.filters:
#                        self.parent.logger.debug(f"Channel {self.chNr}: filter: {f}")
                        filter_done = f.filter(msg)
                        if filter_done:
                            break
                    if not filter_done:
                        if (msg[1]) != 0x04:
                            self.parent.logger.debug(f"Channel {self.chNr} - message dropped: {msg.hex(sep=' ')}")
                        pass
                else:
                    sleep(0)
                    
###################################################################################################
    class DeviceReader:
        """Class representing an ECAN's serial reader"""
        def __init__(self, parent):
            self.cirBufMax = 64
            self.mutex = Lock()
            self.please_quit = False
            self.parent = parent
            self.parent.logger.debug("Ecan device reader constructor")
            # buffer for non-channel msg, ch1, ch2
            #             0               1     2
            self.cirBufs = [collections.deque(maxlen=self.cirBufMax), collections.deque(
                maxlen=self.cirBufMax), collections.deque(maxlen=self.cirBufMax)]

        def try_read(self):
            """ Run in a separate thread !"""
            self.parent.logger.debug("try_read() thread started..")
            while (not self.please_quit):
                inbuf = bytearray(self.parent.port.read(PACKET_LEN))

                if (len(inbuf) == PACKET_LEN) and (inbuf[0] == 0xFF):
                    b = 0
                    if (inbuf[2] == 0x01):
                        b = 1
                    if (inbuf[2] == 0x02):
                        b = 2
                    self.mutex.acquire()
                    self.cirBufs[b].append(inbuf)
                    self.mutex.release()
                elif len(inbuf) == 0:
                    pass
                else:
                    raise Exception("PACKET_LEN error!")
                sleep(0)

        def pop(self, n_buf=0):
            if (n_buf > 2):
                raise Exception("op() error!")
            self.mutex.acquire()
            retVal = bytearray(b'')
            if len(self.cirBufs[n_buf]) != 0:
                retVal = self.cirBufs[n_buf].pop()
            self.mutex.release()
            return retVal

###################################################################################################
    class DeviceWriter:
        """Class representing an ECAN's serial writer"""
        def __init__(self, parent):
            self.cirBufMax = 64
            self.mutex = Lock()
            self.please_quit = False
            self.parent = parent
            self.parent.logger.debug("Ecan device writer constructor")
            self.cirBuf = collections.deque(maxlen=self.cirBufMax)

        def push(self, raw_msg):
            if raw_msg == None:
                self.parent.logger.error("The message is not povided")
                return
            if (len(raw_msg) == 0) or (len(raw_msg) > PACKET_LEN):
                self.parent.logger.error("The message has incorrect length")
                return
            
            self.mutex.acquire()
            self.cirBuf.append(raw_msg)
            self.mutex.release()
            return

        def try_write(self):
            """ Run in a separate thread !"""
            self.parent.logger.debug("try_write() thread started..")
            while (not self.please_quit):
                m = bytearray(b'')
                self.mutex.acquire()
                if len(self.cirBuf) != 0:
                    m = self.cirBuf.pop()
                self.mutex.release()

                if len(m) != 0:
                    self.parent.port.write(m)
                else:
                    sleep(0)
            return
###################################################################################################

    def __init__(self, dev: str, dev_baudrate: int = 115200):
        self.dev = dev
        self.dev_baudrate = dev_baudrate
        self.logger = init_logger()
        self.connected = False
        self.logger.debug(f"ecan constructor: {self.dev}@{self.dev_baudrate}")
        self.port = serial.Serial(port=self.dev, baudrate=self.dev_baudrate, timeout=1)
        self.channel1 = self.EcanChannel(1, self)
        self.channel2 = self.EcanChannel(2, self)
        self.reader = self.DeviceReader(self)
        self.writer = self.DeviceWriter(self)

    def connect(self):
        ecan_id_string = b'\xff\x01\x01\x02ECAN-U01\x00\x00\x00\x00\x00\x00\x00\x00'
        self.port.write(cmds["cmd_connect_ecan"])
        inbuf = self.port.read(PACKET_LEN)
        self.logger.info(f"read {len(inbuf)} bytes")
        self.logger.debug(inbuf)
        if (ecan_id_string != inbuf):
            raise Exception(f"No id string was received from the device at {self.dev}")
        self.connected = True
        self.thread_rd = Thread(target=self.reader.try_read)
        self.thread_rd.start()
        self.thread_wr = Thread(target=self.writer.try_write)
        self.thread_wr.start()

    def disconnect(self):
        self.channel1.close()
        self.channel2.close()
        self.reader.please_quit = True
        self.writer.please_quit = True
        
        self.thread_wr.join(1)
        self.thread_rd.join(1)
        
        if self.thread_rd.is_alive() or self.thread_wr.is_alive():
            self.logger.info("threads are still alive")
        
        ecan_disconnect_string = b'\xff\x0f\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.port.write(cmds["cmd_disconnect_ecan"])
        inbuf = self.port.read(PACKET_LEN)
        self.logger.info(f"read {len(inbuf)} bytes")
        self.logger.debug(inbuf)
        if (ecan_disconnect_string != inbuf):
            self.logger.error(f"No disconnect string was received from the device at {self.dev}")

    def open_channel(self, chan: int, br: str = "0", custom=False, pre: int = 0, tbs1: int = 0, tbs2: int = 0):
        """ The method opens a CAN channel on the device.

        *chan* -- channel number, 1 or 2
        *br*   -- channel baud rate (either one of prefefined or custom)
        *custom* -- if true, the 3 next arguments must be provided
        *pre*, *tbs1*, *tbs2* must be correctly pre-calculated
        """
        if not self.connected:
            raise Exception("Can not open a channel - ECAN is not connected!")
        if (chan == 1):
            self.channel1.open(br, custom, pre, tbs1, tbs2)
        elif (chan == 2):
            self.channel2.open(br, custom, pre, tbs1, tbs2)
        else:
            raise Exception(f"Bad channel ({chan})!")

    def close_channel(self, chan: int):
        if not self.connected:
            raise Exception("Can not close a channel - ECAN is not connected!")
        if (chan == 1):
            self.channel1.close()
        elif (chan == 2):
            self.channel2.close()
        else:
            raise Exception(f"Bad channel ({chan})!")


###############################################################################
if __name__ == "__main__":
    print("The module for the ECAN-U01S USB-CAN bridge.")


########################## E. O. F. ###########################################
