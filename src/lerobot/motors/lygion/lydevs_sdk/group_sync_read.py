#!/usr/bin/env python

from .lydevs_def import *

class GroupSyncRead:
    def __init__(self, ph, start_address, data_length):
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}
        self.id_map = {} 

        self.clearParam()

    def addParam(self, dev_id, motor_type):
        scs_id = map_id(dev_id, motor_type)   

        if scs_id in self.data_dict: 
            return False

        self.data_dict[scs_id] = []  
        self.id_map[dev_id] = []  

        self.is_param_changed = True
        return True

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = list(self.data_dict.keys())  

    def removeParam(self, scs_id):
        if scs_id not in self.data_dict:
            return

        del self.data_dict[scs_id]

        if scs_id in self.id_map:
            del self.id_map[scs_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()
        self.id_map.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict.keys())
        )

    def rxPacket(self):
        self.last_result = True
        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        result, rxpacket = self.ph.syncReadRx(
            self.data_length,
            len(self.data_dict.keys())
        )

        if len(rxpacket) >= (self.data_length + 6):
            for scs_id in self.data_dict:
                self.data_dict[scs_id], result = self.readRx(
                    rxpacket,
                    scs_id,
                    self.data_length
                )
                if result != COMM_SUCCESS:
                    self.last_result = False
        else:
            self.last_result = False
        return result

    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def readRx(self, rxpacket, scs_id, data_length):
        data = []
        rx_length = len(rxpacket)
        rx_index = 0

        while (rx_index + 6 + data_length) <= rx_length:
            headpacket = [0x00, 0x00, 0x00]

            while rx_index < rx_length:
                headpacket[2] = headpacket[1]
                headpacket[1] = headpacket[0]
                headpacket[0] = rxpacket[rx_index]
                rx_index += 1

                if (headpacket[2] == 0xFF) and (headpacket[1] == 0xFF) and headpacket[0] == scs_id:
                    break

            if (rx_index + 3 + data_length) > rx_length:
                break

            if rxpacket[rx_index] != (data_length + 2):
                rx_index += 1
                continue

            rx_index += 1
            Error = rxpacket[rx_index]
            rx_index += 1

            calSum = scs_id + (data_length + 2) + Error
            data = [Error]
            data.extend(rxpacket[rx_index: rx_index + data_length])

            for i in range(0, data_length):
                calSum += rxpacket[rx_index]
                rx_index += 1

            calSum = ~calSum & 0xFF

            if calSum != rxpacket[rx_index]:
                return None, COMM_RX_CORRUPT

            return data, COMM_SUCCESS

        return None, COMM_RX_CORRUPT

    def isAvailable(self, scs_id, address, data_length):
        # if scs_id not in self.data_dict and scs_id not in self.id_map:
        #     return False, 0

        # data = self.data_dict[scs_id]

        if scs_id in self.data_dict:
            real_id = scs_id
        elif scs_id in self.id_map:
            real_id = scs_id + ENCODER_ID_OFFSET
        else:
            return False, 0

        data = self.data_dict.get(real_id, [])

        if (address < self.start_address) or \
           (self.start_address + self.data_length - data_length < address):
            return False, 0

        if not data:
            return False, 0

        base = address - self.start_address + 1

        if len(data) < (base + data_length):
            return False, 0

        return True, data[0]

    def getData(self, scs_id, address, data_length):
        # data = self.data_dict[scs_id]
        if scs_id in self.data_dict:
            real_id = scs_id
        elif scs_id in self.id_map:
            real_id = scs_id + ENCODER_ID_OFFSET
        data = self.data_dict.get(real_id, [])

        base = address - self.start_address + 1

        if data_length == 1:
            return data[base]

        elif data_length == 2:
            return self.ph.scs_makeword(data[base], data[base+1])

        elif data_length == 4:
            return self.ph.scs_makedword(
                self.ph.scs_makeword(data[base], data[base+1]),
                self.ph.scs_makeword(data[base+2], data[base+3])
            )

        return 0