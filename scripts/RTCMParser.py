import socket
import json
from bitstring import ConstBitStream

import os, sys, time
import datetime
import base64
import time
import ssl

'''
MESSAGE TYPES
1001-1004: SATELLITE SPECIFIC MESSAGES INCLUSIVE (GPS)
1005-1006: COMMON MESSAGE 
1007-1008: STRING INCLUSIVE
1009-1012: SATELLITE SPECIFIC MESSAGES INCLUSIVE (GLONASS)
1013: SYSTEM PARAMETER MESSAGE, ID SPECIFIC MESSAGES INCLUSIVE
1014: COMMON MESSAGE
1015-1017: SATELLITE SPECIFIC MESSAGES INCLUSIVE
1019: COMMON MESSAGE
1020: UNIQUE DATATYPE INCLUSIVE (NOT SUPPORTED) - DATA FIELD TYPE CHECK
1021-1022: STRING INCLUSIVE
1023-1027: COMMON MESSAGE
1029: STRING INCLUSIVE
1030-1032: SATELLITE_SPECIFIC MESSAGE INCLUSIVE (GPS)
1033: STRING INCLUSIVE
1034: SATELLITE SPECIFIC MESSAGE INCLUSIVE (GPS NETWORK FKP)
1035: SATELLITE SPECIFIC MESSAGE INCLUSIVE (GLONASS NETWORK FKP)
1037-1039: DATA SPECIFIC MESSAGE INCLUSIVE (GLONASS NETWORK RTK)
1057-1058: SATELLITE SPECIFIC MESSAGE INCLUSIVE (GPS)
1059: SATELLITE SPECIFIC MESSAGE AND CODE SPECIFIC SUB-MESSAGE INCLUSIVE
1060-1062: SATELLITE SPECIFIC MESSAGE INCLUSIVE (SSR GPS)
1063-1064: SATELLITE SPECIFIC MESSAGE INCLUSIVE (SSR GLONASS)
1065: SATELLITE SPECIFIC AND CODE SPECIFIC SUB-MESSAGE INCLUSIVE
1066-1068: SATELLITE SPECIFIC MESSAGE INCLUSIVE (SSR GLONASS)
1230: MESSAGE WITH FLAG (MESSAGE LENGTH VARIABLE)

Parsing result:

<MESSAGE>
+-<DESCRIPTION>
+-<HEADER>
  +-<DATA_FIELD_NAME>
    +-<NAME>
    +-<DATA>
    +-<VALUE>
    +-<UNIT> : optional
+-<CONTENT>
  |   [TYPE 1] : COMMON MESSAGES
  +-<DATA_FIELD_NAME>
  | +-<NAME>
  | +-<DATA>
  | +-<VALUE>
  | +-<UNIT> : optional
  |
  |   [TYPE 2] : SATELLITE SPECIFIC MESSAGES
  +-<SATELLITE_DATA>
  | +-<INDEX>
  |   +-<NAME>
  |   +-<DATA>
  |   +-<VALUE>
  |   +-<UNIT> : optional
  |
  |   [TYPE 3] : STRING INCLUSIVE MESSAGE
  +-<DATA_FIELD_NAME>
  | +-<NAME>
  | +-<DATA>
  | +-<VALUE>
  | +-<UNIT> : optional
  |
  |   [TYPE 4] : SYSTEM PARAMETER MESSAGE
  +-<DATA_FIELD_NAME>
  | +-<NAME>
  | +-<DATA>
  | +-<VALUE>
  | +-<UNIT> : optional 
  +-<PARAMETERS>
  | +-<INDEX>
  |   +-<DATA_FIELD_NAME>
  |     +-<NAME>
  |     +-<DATA>
  |     +-<VALUE>
  |     +-<UNIT> : optional
  | 
  |   [TYPE 5] : CODE BIAS INCLUSIVE MESSAGE
  +-<DATA_FIELD_NAME>
  | +-<NAME>
  | +-<DATA>
  | +-<VALUE>
  | +-<UNIT> : optional
  +-<SATELLITE_DATA>
  | +-<INDEX>
  |   +-<DATA_FIELD_NAME>
  |   | +-<NAME>
  |   | +-<DATA>
  |   | +-<VALUE>
  |   | +-<UNIT>
  |   +-<CODE_BIAS>
  |     +-<INDEX>
  |       +-<DATA_FIELD_NAME>
  |         +-<NAME>
  |         +-<DATA>
  |         +-<VALUE>
  |         +-<UNIT>
  |
  |   [TYPE 6]: GLONASS CODE BIAS MESSAGE
  +-<DATA_FIELD_NAME>
    +-<NAME>
    +-<DATA>
    +-<VALUE>
    +-<UNIT>
  
'''


class RTCMParser:
    def __init__(self, root_pkg_dir, debug=False):
        self.debug = debug
        
        self.root_pkg_dir  = root_pkg_dir
        self.datafield_dir = self.root_pkg_dir + '/RTCM_v3_2/DataField.json'
        self.msg_type_dir  = self.root_pkg_dir + '/RTCM_v3_2/MsgType.json'
        
        self.datafields = None
        self.float_data = None
        
        self.msg_types = None
        self.message_number_list = None
        self.message_group = None
        
        self.rx_buffer = bytearray()
        
        self.count_buf = {}
        
        self.ASCII_TABLE = [
            #  0     1      2      3      4      5      6      7      8      9
            'NUL', 'SOH', 'STX', 'ETX', 'EOT', 'ENQ', 'ACK', 'BEL', 'BS',  'HT',  # 0
            'LF',  'VT',  'FF',  'CR',  'SO',  'SI',  'DLE', 'DC1', 'DC2', 'DC3', # 10
            'DC4', 'NAK', 'SYN', 'ETB', 'CAN', 'EM',  'SUB', 'ESC', 'FS',  'GS',  # 20
            'RS',  'US',  ' ',   '!',   '"',   '#',   '$',   '%',   '&',   "'",   # 30
            '(',   ')',   '*',   '+',   ',',   '-',   '.',   '/',   '0',   '1',   # 40
            '2',   '3',   '4',   '5',   '6',   '7',   '8',   '9',   ':',   ';',   # 50
            '<',   '=',   '>',   '?',   '@',   'A',   'B',   'C',   'D',   'E',   # 60
            'F',   'G',   'H',   'I',   'J',   'K',   'L',   'M',   'N',   'O',   # 70
            'P',   'Q',   'R',   'S',   'T',   'U',   'V',   'W',   'X',   'Y',   # 80
            'Z',   '[',  '\\',   ']',   '^',   '_',   '`',   'a',   'b',   'c',   # 90
            'd',   'e',   'f',   'g',   'h',   'i',   'j',   'k',   'l',   'm',   # 100
            'n',   'o',   'p',   'q',   'r',   's',   't',   'u',   'v',   'w',   # 110
            'x',   'y',   'z',   '{',   '|',   '}',   '~', 'DEL'                  # 120
        ]
        
        self.load_rtcm_data()
        
    def byte2ascii(self, barray):
        ret = ''
        dummy_chr = ''
        
        for b in barray:
            if b == 10:
                ret += '\n'
            elif b == 13:
                ret += '\r'
            elif b >= 32:
                if b < 127:
                    ret += self.ASCII_TABLE[int(b)]
            else:
                ret += dummy_chr
                
        return ret

    def signal_mask_chk(self, num):
        n1 = num // (2**3)
        n2 = (num % (2**3)) // 2**2
        n3 = (num % (2**2)) // 2**1
        n4 = (num % (2**1)) // 2**0
        
        return n1, n2, n3, n4
    
    def load_rtcm_data(self):
        with open(self.datafield_dir) as f:
            data_fields = json.loads(f.read())
            self.datafields = data_fields["DATA_FIELD_INFO"]
            self.float_data = data_fields["FLOAT_DATA_FIELD"]
            
        for k in self.datafields.keys():
            if len(k) != 5 or k[:2] != 'DF':
                print('[RTCM v3.2] Key error: Data field has incorrect key : {}'.format(k))
                return False
            
        print('[RTCM v3.2] Data field info. loaded.')
        
        with open(self.msg_type_dir) as f:
            message_types = json.loads(f.read())
            self.message_number_list = message_types["MESSAGE_NUMBER_LIST"]
            self.message_group = message_types["MESSAGE_GROUP"]
            self.msg_types = message_types["MESSAGE_STRUCTURE"]
        
        if k in self.msg_types.keys():
            if len(k) != 4 or k[0] != '1':
                print("key error: Incorrect message number found : {}".format(k))
                return False
        
        print('[RTCM v3.2] Message info. loaded.')
        
        return True
    
    def load_msg_metadata(self, message_number):
        return self.msg_type["{}".format(message_number)]
        
    def get_masked_index(self, num, size):
        l = []
        for i in range(size):
            if (num >> (size - i - 1)) & 0x1:
                l.append(l)
            else:
                continue
                
        return l
    
    def parse_msg(self, barray):
        '''
        PREAMBLE : 8 bit < 11010011 >
        RESERVED : 6 bit < 000000 >
        '''
        
        tmp_msg = ConstBitStream(barray)
        msg_length = len(barray)
        bit_stream_length = len(tmp_msg)

        cursor = 0
        stride = 8
        preamble = tmp_msg.read('bin:{}'.format(stride))
        cursor += stride
        
        stride = 6
        reserved = tmp_msg.read('bin:{}'.format(stride))
        cursor += stride
        
        stride = 10
        message_length = tmp_msg.read('uint:{}'.format(stride))
        cursor += stride
        
        stride = 12
        message_number = tmp_msg.read('uint:{}'.format(stride))
        
        if self.debug:
            print('[RTCM v3.2] Message received.')
            print('MSG: {}'.format(tmp_msg))
            print('MSG LENGTH: {} bytes, {} bits'.format(msg_length, bit_stream_length))
            print('MSG HEADER: {}'.format(preamble, reserved, message_length))
            print('MSG PAYLOAD LENGTH: {} bits'.format(bit_stream_length - 48))
            print('MSG TYPE: {}'.format(message_number))
            
        if message_number in self.count_buf.keys():
            self.count_buf[message_number] += 1
        else:
            self.count_buf[message_number] = 1
            
        message = None
        
        if message_number in self.message_number_list:
            packet = tmp_msg[cursor:]
            
            if message_number in self.message_group["GPS_SATELLITE_SPECIFIC_MESSAGE"]:
                message = self.parse_satellite_specific_message(message_number, packet)
            elif message_number in self.message_group["GLONASS_SATELLITE_SPECIFIC_MESSAGE"]:
                message = self.parse_satellite_specific_message(message_number, packet)
            elif message_number in self.message_group["COMMON_MESSAGE"]:
                message = self.parse_common_message(message_number, packet)
            elif message_number in self.message_group["STRING_INCLUSIVE_MESSAGE"]:
                message = self.parse_string_inclusive_message(message_number, packet)
            elif message_number in self.message_group["SYSTEM_PARAMETER_MESSAGE"]:
                message = self.parse_system_parameter_message(packet)
            elif message_number in self.message_group["CODE_BIAS_INCLUSIVE_MESSAGE"]:
                message = self.parse_code_bias_inclusive_message(message_number, packet)
            elif message_number in self.message_group["GLONASS_CODE_BIAS_MESSAGE"]:
                message = self.parse_glonass_code_phase_bias_message(packet)
            elif message_number in self.message_group["MSM"]:
                message = self.parse_msm_message(message_number, packet)
                
        else:
            print('Parsing Error: Unknown message number - {}'.format(message_number))
            
        return message
    
    def translate_message(self, message):
        header = message["HEADER"]
        content = message["CONTENT"]
        
        res = ''
        for k in header.keys():
            res += '[{}] {}: {}'.format(k, header[k]["NAME"], header[k]["VALUE"])
            if "UNIT" in header[k].keys():
                res += ' {}'.format(header[k]["UNIT"])

            res += '\r\n'
            
        for k in content.keys():
            if k == "SATELLITE_DATA":
                res += '<SATELLITE SPECIFIC MESSAGE>\r\n'
                for idx in content["SATELLITE_DATA"].keys():
                    res += '    (SATELLITE INDEX: {})\r\n'.format(idx)
                    for kk in content["SATELLITE_DATA"][idx].keys():
                        if kk == "CODE_BIAS":
                            res += '    <CODE BIASES>\r\n'
                            for sub_idx in content["SATELLITE_DATA"][idx]["CODE_BIAS"]:
                                res += '        (CODE BIAS INDEX: {})\r\n'.format(sub_idx)
                                for kkk in content["SATELLITE_DATA"][idx]["CODE_BIAS"][sub_idx].keys():
                                    res += '        [{}] {}: {}'.format(kkk,
                                                                            content["SATELLITE_DATA"][idx]["CODE_BIAS"][sub_idx]["NAME"],
                                                                            content["SATELLITE_DATA"][idx]["CODE_BIAS"][sub_idx]["VALUE"])
                                    if "UNIT" in content["SATELLITE_DATA"][idx]["CODE_BIAS"][sub_idx].keys():
                                        res += ' {}'.format(content["SATELLITE_DATA"][idx]["CODE_BIAS"][sub_idx]["UNIT"])
                                    res += '\r\n'
                        else:
                            res += '    [{}] {}: {}'.format(kk,
                                                                content["SATELLITE_DATA"][idx][kk]["NAME"], 
                                                                content["SATELLITE_DATA"][idx][kk]["VALUE"])
                            if "UNIT" in content["SATELLITE_DATA"][idx][kk].keys():
                                res += ' {}'.format(content["SATELLITE_DATA"][idx][kk]["UNIT"])
                            res += '\r\n'
                            
            elif k == "PARAMETERS":
                res += '<PARAMETERS>\r\n'
                for idx in content["PARAMETERS"].keys():
                    res += '    (PARAMETER INDEX: {})\r\n'.format(idx)
                    for kk in content["SATELLITE_DATA"][idx].keys():
                        res += '    [{}] {}: {}'.format(kk,
                                                        content["PARAMETERS"][idx][kk]["NAME"],
                                                        content["PARAMETERS"][idx][kk]["VALUE"])
                        if "UNIT" in content["PARAMETERS"][idx][kk].keys():
                            res += ' {}'.format(content["PARAMETERS"][idx][kk]["UNIT"])
                        
                        res += '\r\n'
                        
            elif k == "MSM_DATA":
                res += '<SATELLITE SPECIFIC MSM>\r\n'
                for idx in content["MSM_DATA"].keys():
                    res += '    (SATELLITE INDEX: {})\r\n'.format(idx)
                    for kk in content["MSM_DATA"][idx].keys():
                        res += '    [{}] {}: {}'.format(kk, content["MSM_DATA"][idx][kk]["NAME"], content["MSM_DATA"][idx][kk]["VALUE"])
                        if "UNIT" in content["MSM_DATA"][idx][kk].keys():
                            res += '{}'.format(content["MSM_DATA"][idx][kk]["UNIT"])
                        res += '\r\n'
                        
            else:
                # print(k)
                if k == "CODE_BIAS":
                    for kk in content["CODE_BIAS"]:
                        res += '[{}] {}: {}'.format(kk, content["CODE_BIAS"][kk]["NAME"], content["CODE_BIAS"][kk]["VALUE"])
                        if "UNIT" in content["CODE_BIAS"][kk].keys():
                            res += '{}'.format(content["CODE_BIAS"][kk]["UNIT"])
                        res += '\r\n'
                else:
                    res += '[{}] {}: {}'.format(k, content[k]["NAME"], content[k]["VALUE"])
                    if "UNIT" in content[k].keys():
                        res += ' {}'.format(content[k]["UNIT"])
                    res += '\r\n'
                
        print(res)
         
    def parse_satellite_specific_message(self, message_number, packet):
        '''
        GPS: 1001, 1002, 1003, 1004, 1015, 1016, 1017, 1030, 1034, 1057, 1058, 1060, 1061, 1062
        GLONASS: 1009, 1010, 1011, 1012, 1031, 1035, 1037, 1038, 1039, 1063, 1064, 1066, 1067, 1068
        '''
        msg_metadata = self.load_msg_metadata(message_number)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        satellite_data = {}
        n_satellites = 0
        ret["DESCRIPTION"] = self.msg_types["{}".format(message_number)]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            if k in self.float_data:
                data = float(data)
            if k in ["DF006", "DF035"]:
                n_satellites = data
                
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {"NAME": field_name,
                    "DATA": data,
                    "VALUE": value}
        
            if unit is not None:
                header[k]["UNIT"] = unit
            
        for i in range(n_satellites):
            tmp_content = {}
            unit = None
            
            for k in msg_metadata["CONTENT"]:
                field_name = self.datafields[k]["NAME"]
                dtype = self.datafields[k]["DTYPE"]
                data = msg.read(dtype)
                if k in self.float_data:
                    data = float(data)
                
                unit = None
                if "SCALE" in self.datafields[k].keys():
                    value = data * self.datafields[k]["SCALE"]
                elif "VALUE" in self.datafields[k].keys():
                    value = self.datafields[k]["VALUES"][data]
                else:
                    value = data
                if "UNIT" in self.datafields[k].keys():
                    unit = self.datafields[k]["UNIT"]
                
                tmp_content[k] = {"NAME": field_name,
                                  "DATA": data,
                                  "VALUE": value}

                if unit is not None:
                    tmp_content[k]["UNIT"] = unit
                    
            satellite_data[i] = tmp_content
        
        contents["SATELLITE_DATA"] = satellite_data
        
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_common_message(self, message_number, packet):
        '''
        1005, 1006, 1014, 1019, 1020, 1023, 1024, 1025, 1026, 1027, 1032
        '''
        msg_metadata = self.load_msg_metadata(message_number)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        ret["DESCRIPTION"] = self.msg_types["{}".format(message_number)]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
                
            if k in self.float_data:
                data = float(data)
                
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {"NAME": field_name,
                    "DATA": data,
                    "VALUE": value}
        
            if unit is not None:
                header[k]["UNIT"] = unit
            
       
        for k in msg_metadata["CONTENT"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k in self.float_data:
                data = float(data)
            
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUE" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
            
            contents[k] = {"NAME": field_name,
                           "DATA": data,
                           "VALUE": value}
            
            if unit is not None:
                contents[k]["UNIT"] = unit
            
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_string_inclusive_message(self, message_number, packet):
        '''
        1007, 1008, 1021, 1022, 1029, 1033
        "
        '''
        msg_metadata = self.load_msg_metadata(message_number)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        
        ret["DESCRIPTION"] = self.msg_types["{}".format(message_number)]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k in self.float_data:
                data = float(data)
            
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
            
            header[k] = {"NAME": field_name,
                      "DATA": data,
                      "VALUE": value}
            
            if unit is not None:
                header[k]["UNIT"] = unit
        
        for k in msg_metadata["CONTENT"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]['DTYPE']
            if k in ["DF030", "DF033", "DF140", "DF144", "DF146", "DF228", "DF230", "DF232"]:
                data = []
                for i in range(str_length):
                    data.append(msg.read(dtype))
                data = self.byte2ascii(data)
            else:
                data = msg.read(dtype)
            str_length = 0
            '''
            STRING_LENGTH : DF029, DF032, DF139, DF143, DF145, DF227, DF229, DF231
            CHR:            DF030, DF033, DF140, DF144, DF146, DF228, DF230, DF232
            '''
            if k in ["DF029", "DF032", "DF139", "DF143", "DF145", "DF227", "DF229", "DF231"]:
                str_length = data
                
            elif k in self.float_data:
                data = float(data)
            
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            contents[k] = {"NAME": field_name,
                      "DATA": data,
                      "VALUE": value}
            
            if unit is not None:
                contents[k]["UNIT"] = unit
            
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_system_parameter_message(self, packet):
        msg_metadata = self.load_msg_metadata(1013)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        n_ids = 0
        ret["DESCRIPTION"] = self.msg_types["1013"]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k in self.float_data:
                data = float(data)
            
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {"NAME": field_name,
                      "DATA": data,
                      "VALUE": value}
            
            if unit is not None:
                header[k]["UNIT"] = unit
            
        for k in msg_metadata["CONTENT"]:
            if k == "DF055":
                break
            
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k == "DF053":
                n_ids = data            
            elif k in self.float_data:
                data = float(data)
            
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
                
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            contents[k] = {"NAME": field_name,
                           "DATA": data,
                           "VALUE": value}
            
            if unit is not None:
                contents[k]["UNIT"] = unit
        
        parameters = {}
        for i in range(n_ids):
            parameter = {}
            for k in ["DF055", "DF056", "DF057"]:
                field_name = self.datafields[k]["NAME"]
                dtype = self.datafields[k]["DTYPE"]
                
                data = msg.read(dtype)
                
                if k in self.float_data:
                    data = float(data)
                
                unit = None
                if "SCALE" in self.datafields[k].keys():
                    value = data * self.datafields[k]["SCALE"]
                elif "VALUES" in self.datafields[k].keys():
                    value = self.datafields[k]["VALUES"][data]
                else:
                    value = data
                    
                if "UNIT" in self.datafields[k].keys():
                    unit = self.datafields[k]["UNIT"]
                    
                parameter[k] = {"NAME": field_name,
                                "DATA": data,
                                "VAULE": value}
                
                if unit is not None:
                    parameter[k]["UNIT"] = unit
            parameters[i] = parameter
                    
        contents["PARAMETERS"] = parameters
        
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_code_bias_inclusive_message(self, message_number, packet):
        '''
        1059, 1065
        DF387,
        DF379,
        '''
        msg_metadata = self.load_msg_metadata(message_number)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        satellite_data = {}
        n_satellites = 0
        
        ret["DESCRIPTION"] = self.msg_types["{}".format(message_number)]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k == "DF387":
                n_satellites = data
            elif k in self.float_data:
                data = float(data)
                
            unit = None
            
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VLAUES"][data]
            else:
                value = data
                
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {"NAME": field_name, 
                         "DATA": data,
                         "VALUE": value}
            
            if unit is not None:
                header[k]["UNIT"] = unit
                
        for i in n_satellites:
            tmp_content = {}
            code_biases = {}
            n_code_biases = 0
            
            for k in msg_metadata["CONTENT"]:
                field_name = self.datafields[k]["NAME"]
                dtype = self.datafields[k]["DTYPE"]
                data = msg.read(dtype)
                
                if k == "DF379":
                    n_code_biases = data
                elif k in self.float_data:
                    data = float(data)
                
                unit = None
                
                if "SCALE" in self.datafields[k].keys():
                    value = data * self.datafields[k]["SCALE"]
                elif "VALUES" in self.datafields[k].keys():
                    value = self.datafields[k]["VALUES"][data]
                else:
                    value = data
                
                if "UNIT" in self.datafields[k].keys():
                    unit = self.datafields[k]["UNIT"]
                    
                tmp_content[k] = {"NAME": field_name,
                                  "DATA": data,
                                  "VALUE": value}
                
                if unit is not None:
                    tmp_content[k]["UNIT"] = unit
            # contents[i] = tmp_content
                    
            for j in n_code_biases:
                code_bias = {}
                for kk in msg_metadata["SUBCONTENT"]:
                    field_name = self.datafields[kk]["NAME"]
                    dtype= self.datafields[kk]["DTYPE"]
                    data = msg.read(dtype)
                    if kk in self.float_data:
                        data = float(data)
                    
                    unit = None
                    
                    if "SCALE" in self.datafields[kk].keys():
                        value = data * self.datafields[kk]["SCALE"]
                    elif "VALUES" in self.datafields[kk].keys():
                        value = self.datafields[kk]["VALUES"][data]
                    else:
                        value = data
                        
                    if "UNIT" in self.datafields[kk].keys():
                        unit = self.datafields[kk]["UNIT"]
                        
                    code_bias[kk] = {"NAME": field_name,
                                      "DATA": data, 
                                      "VALUE": value}
                    
                    if unit is not None:
                        code_bias[kk]["UNIT"] = unit
                code_biases[j] = code_bias
            tmp_content["CODE_BIAS"] = code_biases
            
            satellite_data[i] = tmp_content
        
        contents["SATELLITE_DATA"] = satellite_data
            
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_glonass_code_phase_bias_meessage(self, packet):
        '''
        1230
        '''
        msg_metadata = self.load_msg_metadata(1230)
        msg = ConstBitStream(packet)
        
        ret = {}
        header = {}
        contents = {}
        code_biases = {}
        ret["DESCRIPTION"] = self.msg_types["1230"]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
        
            if k in self.float_data:
                data = float(data)
                
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {"NAME": field_name,
                         "DATA": data,
                         "VALUE": value}
            
            if unit is not None:
                header[k]["UNIT"] = unit
            
        signal_mask = 0
        for k in msg_metadata["CONTENT"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            
            if k in self.float_data:
                data = float(data)
            
            if k == "DF422":
                signal_mask = data
                
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
                
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
            
            contents[k] = {"NAME": field_name,
                           "DATA": data,
                           "VALUE": value}
            
            if unit is not None:
                contents[k]["UNIT"] = unit

        l1_ca, l1_p, l2_ca, l2_p = self.signal_mask_chk(signal_mask)
        
        # GLONASS L1 C/A Code-Phase Bias
        k = "DF423"
        field_name = self.datafields[k]["NAME"]
        unit = self.datafields[k]["UNIT"]
        
        if l1_ca:
            dtype = self.datafields[k]["DTYPE"]    
            data = msg.read(dtype)
            value = float(data) * self.datafields[k]["SCALE"]
        else:
            data = 0
            value = data
        code_biases[k] = {"NAME": field_name,
                          "DATA": data,
                          "VALUE": value,
                          "UNIT": unit}
            
        # GLONASS L1 P Code-Phase Bias
        k = "DF424"
        field_name = self.datafields[k]["NAME"]
        unit = self.datafields[k]["UNIT"]
        
        if l1_p:
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            value = float(data) * self.datafields[k]["SCALE"]
        else:
            data = 0
            value = data
            
        code_biases[k] = {"NAME": field_name,
                          "DATA": data,
                          "VALUE": value,
                          "UNIT": unit}
        
        # GLONASS L2 C/A Code-Phase Bias
        k = "DF425"
        field_name = self.datafields[k]["NAME"]
        unit = self.datafields[k]["UNIT"]
        
        if l2_ca:
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            value = float(data) * self.datafields[k]["SCALE"]
        else:
            data = 0
            value = data
        
        code_biases[k] = {"NAME": field_name,
                          "DATA": data,
                          "VALUE": value,
                          "UNIT": unit}
        
        # GLONASS L2 P Code-Phase Bias
        k = "DF426"
        field_name = self.datafields[k]["NAME"]
        unit = self.datafields[k]["UNIT"]
        
        if l2_p:
            dtype = self.datafields[k]["DTYPE"]
            data = msg.read(dtype)
            value = float(data) * self.datafields[k]["SCALE"]
        else:
            data = 0
            value = data
            
        code_biases[k] = {"NAME": field_name,
                          "DATA": data,
                          "VALUE": value,
                          "UNIT": unit}
        
        contents["CODE_BIAS"] = code_biases
        
        ret["HEADER"] = header
        ret["CONTENT"] = contents
        
        return ret
    
    def parse_msm_message(self, message_number, packet):
        msg_metadata = self.load_msg_metadata(message_number)
        msg = ConstBitStream(packet)
        msg_number_str = '{}'.format(message_number)
        ret = {}
        header = {}
        contents = {}
        
        satellite_ids = []
        signal_ids = []
        n_satellites = 0
        n_signals = 0
        
        ret["DESCRIPTION"] = self.msg_types[msg_number_str]["DESCRIPTION"]
        
        for k in msg_metadata["HEADER"]:
            field_name = self.datafields[k]["NAME"]
            dtype = self.datafields[k]["DTYPE"]
            if k == "DF001":
                data += int(msg.read(dtype))
            elif k == "DF394":
                data = int(msg.read(dtype))
                satellite_ids = self.get_masked_index(data, 64)
                n_satellites = len(satellite_ids)
            elif k == "DF395":
                data = int(msg.read(dtype))
                signal_ids = self.get_masked_index(data, 32)
                n_signals = len(signal_ids)
            else:
                data = msg.read(dtype)
                
            if k in self.float_data:
                data = float(data)
                
            unit = None
            if "SCALE" in self.datafields[k].keys():
                value = data * self.datafields[k]["SCALE"]
            elif "VALUES" in self.datafields[k].keys():
                value = self.datafields[k]["VALUES"][data]
            else:
                value = data
            if "UNIT" in self.datafields[k].keys():
                unit = self.datafields[k]["UNIT"]
                
            header[k] = {
                "NAME": field_name,
                "DATA": data,
                "VALUE": value
            }

            if unit is not None:
                header[k]["UNIT"] = unit
        msm_contents = {}
        for i in range(n_satellites):
            tmp_content = {}
            for k in msg_metadata["CONTENT_SATELLITE"]:
                field_name = self.datafields[k]["NAME"]
                dtype = self.datafields[k]["DTYPE"]
                data = msg.read(dtype)
                
                if k in self.float_data:
                    data = float(data)
                    
                unit = None
                if "SCALE" in self.datafields[k].keys():
                    value = data * self.datafields[k]["SCALE"]
                elif "VALUES" in self.datafields[k].keys():
                    value = self.datafields[k]["VALUES"][data]
                else:
                    value = data
                if "UNIT" in self.datafields[k]["UNIT"]:
                    unit = self.datafields[k]["UNIT"]
                
                tmp_content[k] = {
                    "NAME": field_name,
                    "DATA": data,
                    "VALUE": value
                }
                
                if unit is not None:
                    tmp_content[k]["UNIT"] = unit
                    
                for kk in msg_metadata["CONTENT_SIGNAL"]:
                    field_name = self.datafields[kk]["NAME"]
                    dtype = self.datafields[kk]["DTYPE"]
                    data = msg.read(dtype)
                    
                    if k in self.float_data:
                        data = float(data)
                        
                    unit = None
                    if "SCALE" in self.datafields[kk].keys():
                        value = data * self.datafields[kk]["SCALE"]
                    elif "VALUES" in self.datafields[kk].keys():
                        value = self.datafields[kk]["VALUES"][data]
                    else:
                        value = data
                    if "UNIT" in self.datafields[kk]["UNIT"]:
                        unit = self.datafields[kk]["UNIT"]
                        
                    tmp_content[kk] = {
                        "NAME": field_name, 
                        "DATA": data,
                        "VALUE": value
                    }
                    
                    if unit is not None:
                        tmp_content[kk]["UNIT"] = unit
                        
                msm_contents['{}'.format(satellite_ids[i])] = tmp_content
                
        ret["HEADER"] = header
        contents["MSM_DATA"] = msm_contents
        ret["CONTENT"] = contents
        
        return ret