#!/usr/bin/env python
# 

import binascii
import sys
from datetime import datetime, timedelta
from gnuradio import gr

class mac(gr.basic_block):
    # The numbers represents bytes location/width
    MAC_FRAME_MFH_WIDTH = 2
    MAC_FRAME_MFH_OFFSET = 0
    MAC_FRAME_ODA_WIDTH = 6
    MAC_FRAME_ODA_OFFSET = 2
    MAC_FRAME_OSA_WIDTH = 6
    MAC_FRAME_OSA_OFFSET = 8
    MAC_FRAME_ETHERTYPE_OR_LENGTH_WIDTH = 2
    MAC_FRAME_PAYLOAD_OFFSET = MAC_FRAME_MFH_WIDTH + MAC_FRAME_ODA_WIDTH + MAC_FRAME_OSA_WIDTH + MAC_FRAME_ETHERTYPE_OR_LENGTH_WIDTH
    MAC_FRAME_ICV_WIDTH = 4

    PHY_BLOCK_HEADER_WIDTH = 4
    PHY_BLOCK_BODY_OFFSET = 4
    PHY_BLOCK_PBCS_WIDTH = 4 

    # The numbers represents bits location/width
    PHY_BLOCK_HEADER_SSN_WIDTH = 16
    PHY_BLOCK_HEADER_SSN_OFFSET = 0
    PHY_BLOCK_HEADER_MFBO_WIDTH = 9
    PHY_BLOCK_HEADER_MFBO_OFFSET = 16
    PHY_BLOCK_HEADER_VPBF_WIDTH = 1
    PHY_BLOCK_HEADER_VPBF_OFFSET = 25
    PHY_BLOCK_HEADER_MMQF_WIDTH = 1
    PHY_BLOCK_HEADER_MMQF_OFFSET = 26
    PHY_BLOCK_HEADER_MMBF_WIDTH = 1
    PHY_BLOCK_HEADER_MMBF_OFFSET = 27
    PHY_BLOCK_HEADER_OPSF_WIDTH = 1
    PHY_BLOCK_HEADER_OPSF_OFFSET = 28

    MAX_SEGMENTS = 3 # max number of segments (PHY blocks) in one MAC frames
    MAX_FRAMES_IN_BUFFER = 10 # max number of MAC frames in tx buffer
    rx_incomplete_frames = {}
    tx_frames_buffer = {}
    tx_frames_in_buffer = 0
    last_ssn = -1 # track last ssn received
    need_to_send_status = True
    last_sound_frame = datetime.min # last time a sound frame was transmitted
    sound_frame_rate = 5 ; # minimum time in seconds between sounds frames
    phy_ready = False

    """
    docstring for block mac
    """
    def __init__(self, device_addr, debug):
        gr.basic_block.__init__(self,
            name="mac",
            in_sig=[],
            out_sig=[])
        self.message_port_register_out(gr.pmt.to_pmt("phy out"))
        self.message_port_register_in(gr.pmt.to_pmt("phy in"))
        self.message_port_register_out(gr.pmt.to_pmt("app out"))
        self.message_port_register_in(gr.pmt.to_pmt("app in"))
        self.set_msg_handler(gr.pmt.to_pmt("app in"), self.app_in_handler)
        self.set_msg_handler(gr.pmt.to_pmt("phy in"), self.phy_in_handler)
        self.device_addr = bytearray(''.join(chr(x) for x in device_addr))
        self.debug = debug

    def app_in_handler(self, msg):
        if gr.pmt.is_pair(msg):
            if gr.pmt.is_u8vector(gr.pmt.cdr(msg)) and gr.pmt.is_dict(gr.pmt.car(msg)):
                payload = bytearray(gr.pmt.u8vector_elements(gr.pmt.cdr(msg)))
                if self.debug: print "MAC: received payload from APP, size = " + str(len(payload)) + ", frames in buffer = " + str(self.tx_frames_in_buffer)
                if self.tx_frames_in_buffer >= self.MAX_FRAMES_IN_BUFFER:
                    sys.stderr.write ("MAC: buffer is full, dropping frame from APP\n")
                    self.need_to_send_status = True
                    return
                else:
                    dict = gr.pmt.to_python(gr.pmt.car(msg))
                    dest = bytearray(dict["dest"])
                    mac_frame = self.create_mac_frame(dest, payload)
                    self.submit_mac_frame(mac_frame)
                    self.send_frame_to_phy()
                    self.need_to_send_status = True
                    self.send_status_to_app()

    def phy_in_handler(self, msg):
        if gr.pmt.is_pair(msg):
            cdr = gr.pmt.cdr(msg)
            car = gr.pmt.car(msg)
            if gr.pmt.is_dict(car) and gr.pmt.is_u8vector(cdr):
                dic = gr.pmt.to_python(car)
                if dic["type"] == "sof":
                    if self.debug: print "MAC: received MPDU (SOF) from PHY"
                    self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.to_pmt("receive"));
                    mpdu_payload = bytearray(gr.pmt.u8vector_elements(cdr))
                    self.parse_mpdu_payload(mpdu_payload)
                elif dic["type"] == "sound":
                    if self.debug: print "MAC: received MPDU (Sound) from PHY"
                    self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.to_pmt("sense"));
        if gr.pmt.is_symbol(msg):
            status = gr.pmt.symbol_to_string(msg)
            if status == "READY":
                self.phy_ready = True
                self.send_frame_to_phy()

    def send_frame_to_phy(self):
        if not self.phy_ready: return

        # dict
        dict = gr.pmt.make_dict();

        if (datetime.now()-self.last_sound_frame).total_seconds() >= self.sound_frame_rate:
            dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("type"), gr.pmt.to_pmt("sound"));
            mpdu_payload_u8vector = gr.pmt.init_u8vector(0, []);
            if self.debug: print "MAC: sending MPDU (Sound) to PHY"
            self.last_sound_frame = datetime.now();
        else:
            dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("type"), gr.pmt.to_pmt("sof"));
            mpdu_payload = self.create_mpdu_payload()
            if not mpdu_payload: return
            # create u8vector        
            mpdu_payload_u8vector = gr.pmt.init_u8vector(len(mpdu_payload), list(mpdu_payload))
            if self.debug: print "MAC: sending MPDU (SOF) to PHY"
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(dict, mpdu_payload_u8vector));
        self.phy_ready = False
        self.send_status_to_app()

    def send_status_to_app(self):
        if self.need_to_send_status:
            if self.tx_frames_in_buffer < self.MAX_FRAMES_IN_BUFFER:
                self.message_port_pub(gr.pmt.to_pmt("app out"), gr.pmt.to_pmt("READY"));
                self.need_to_send_status = False

    def forecast(self, noutput_items, ninput_items_required):
        #setup size of input_items[i] for work call
        for i in range(len(ninput_items_required)):
            ninput_items_required[i] = noutput_items

    def start(self):
        self.send_status_to_app()

    def general_work(self, input_items, output_items):
        noutput_items = output_items[0].size
        ninput_items = input_items[0].size
        n = min(noutput_items, ninput_items)
        self.consume(0, len(input_items[0]))
        #self.consume_each(len(input_items[0]))
        return n

    def create_mac_frame(self, dest, payload):
        overhead_size = self.MAC_FRAME_PAYLOAD_OFFSET + self.MAC_FRAME_ICV_WIDTH
        mac_frame = bytearray(overhead_size + len(payload))
        offset = 0
        mfl = len(mac_frame) - self.MAC_FRAME_MFH_WIDTH - self.MAC_FRAME_ICV_WIDTH - 1 # calculate frame length field
        header = bytearray(2) # header is 2 bytes
        header[0] = (mfl & 0x3F) << 2 | 0b01  # concat frame legnth (14 bits) with frame type (2 bits)
        header[1] = (mfl & 0x3FC0) >> 6

        self.set_bytes_field(mac_frame, header, self.MAC_FRAME_MFH_OFFSET)
        self.set_bytes_field(mac_frame, self.device_addr, self.MAC_FRAME_OSA_OFFSET)
        self.set_bytes_field(mac_frame, dest, self.MAC_FRAME_ODA_OFFSET)
        self.set_bytes_field(mac_frame, payload, self.MAC_FRAME_PAYLOAD_OFFSET)
        
        crc = self.crc32(mac_frame[self.MAC_FRAME_MFH_OFFSET + self.MAC_FRAME_MFH_WIDTH:-self.MAC_FRAME_ICV_WIDTH]) # calculate crc excluding MFH and ICV fields
        self.set_bytes_field(mac_frame, crc, self.MAC_FRAME_PAYLOAD_OFFSET + len(payload))

        return mac_frame

    def parse_mac_frame(self, mac_frame):
        header = self.get_bytes_field(mac_frame, self.MAC_FRAME_MFH_OFFSET, self.MAC_FRAME_MFH_WIDTH)
        length = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + self.MAC_FRAME_MFH_WIDTH + self.MAC_FRAME_ICV_WIDTH + 1
        payload_size = length - self.MAC_FRAME_PAYLOAD_OFFSET - self.MAC_FRAME_ICV_WIDTH
        source_addr = self.get_bytes_field(mac_frame, self.MAC_FRAME_OSA_OFFSET, self.MAC_FRAME_OSA_WIDTH)
        dest_addr = self.get_bytes_field(mac_frame, self.MAC_FRAME_ODA_OFFSET, self.MAC_FRAME_ODA_WIDTH)
        payload = self.get_bytes_field(mac_frame, self.MAC_FRAME_PAYLOAD_OFFSET, payload_size)        
        if not self.crc32_check(mac_frame[self.MAC_FRAME_MFH_OFFSET + self.MAC_FRAME_MFH_WIDTH:]):
            sys.stderr.write("MAC: MAC frame CRC error\n")
        return payload

    def submit_mac_frame(self, frame):
        dest = self.get_bytes_field(frame, self.MAC_FRAME_ODA_OFFSET, self.MAC_FRAME_ODA_WIDTH)
        if (str(dest) in self.tx_frames_buffer):
            stream = self.tx_frames_buffer[str(dest)]
        else:
            stream = {"ssn": 0, "frames": [], "remainder": bytearray(0)}
            self.tx_frames_buffer[str(dest)] = stream
        stream["frames"].append(frame)
        self.tx_frames_in_buffer += 1
        if self.debug: print "MAC: added MAC frame to tranmission buffer, frame size = " + str(len(frame))

    def receive_mac_frame(self, frame):
        payload = self.parse_mac_frame(frame)
        if payload:
            if self.debug: print "MAC: received MAC frame, size = " + str(len(frame)) + ", sending payload to APP"
            # dict
            pmt_dict = gr.pmt.make_dict();
            #dict = pmt::dict_add(dict, pmt::mp("crc_included"), pmt::PMT_T);

            # create u8vector        
            payload_u8vector = gr.pmt.init_u8vector(len(payload), list(payload))

            # mpdu
            self.message_port_pub(gr.pmt.to_pmt("app out"), gr.pmt.cons(pmt_dict, payload_u8vector));

    def init_phy_block(self, size, ssn, mac_boundary_offset, valid_flag, message_queue_flag, mac_boundary_flag, oldest_ssn_flag):
        phy_block = bytearray(size + 8)
        self.set_numeric_field(phy_block, ssn, self.PHY_BLOCK_HEADER_SSN_OFFSET, self.PHY_BLOCK_HEADER_SSN_WIDTH)
        if mac_boundary_flag == 1:
            self.set_numeric_field(phy_block, mac_boundary_offset, self.PHY_BLOCK_HEADER_MFBO_OFFSET, self.PHY_BLOCK_HEADER_MFBO_WIDTH)
        self.set_numeric_field(phy_block, valid_flag, self.PHY_BLOCK_HEADER_VPBF_OFFSET, self.PHY_BLOCK_HEADER_VPBF_WIDTH)
        self.set_numeric_field(phy_block, message_queue_flag, self.PHY_BLOCK_HEADER_MMQF_OFFSET, self.PHY_BLOCK_HEADER_MMQF_WIDTH)
        self.set_numeric_field(phy_block, mac_boundary_flag, self.PHY_BLOCK_HEADER_MMBF_OFFSET, self.PHY_BLOCK_HEADER_MMBF_WIDTH)
        self.set_numeric_field(phy_block, oldest_ssn_flag, self.PHY_BLOCK_HEADER_OPSF_OFFSET, self.PHY_BLOCK_HEADER_OPSF_WIDTH)
        return phy_block

    def create_mpdu_payload(self):
        # Check if buffer is empty
        if not len(self.tx_frames_buffer): return
        stream = {}
        for key in self.tx_frames_buffer:
            if self.tx_frames_buffer[key]["frames"] or self.tx_frames_buffer[key]["remainder"]:
                stream = self.tx_frames_buffer[key]
                break
        if not stream: return

        frames = stream["frames"]
        ssn = stream["ssn"]
        remainder = stream["remainder"]
        num_segments = 0
        payload = bytearray(0)
        end_of_stream = False

        while num_segments < self.MAX_SEGMENTS:
            # Creating a new segment
            mac_boundary_flag = False
            mac_boundary_offset = 0
            body_size = 512
            segment = remainder[0:body_size]
            if remainder:
                remainder = remainder[body_size:]
                if not remainder: self.tx_frames_in_buffer -= 1
            while len(segment) < body_size and frames:
                if not mac_boundary_flag:
                    mac_boundary_offset = len(segment)
                    mac_boundary_flag = True
                remainder = frames[0][body_size-len(segment):]
                segment += frames[0][0:body_size-len(segment)]                
                frames.pop(0)
                if not remainder: self.tx_frames_in_buffer -= 1

            # Determine if segment should be 128 bytes or 512
            if len(segment) < body_size:
                if len(segment) < 128 and num_segments == 0: body_size = 128 
                if not mac_boundary_flag:
                    mac_boundary_offset = len(segment)
                    mac_boundary_flag = True
                segment[len(segment):body_size] = bytearray(body_size-len(segment))
                
            # Creating the PHY block
            phy_block = self.init_phy_block(body_size, ssn, mac_boundary_offset, True, False, mac_boundary_flag, False) # Setting header fields
            self.set_bytes_field(phy_block, segment, self.PHY_BLOCK_BODY_OFFSET) # assigning body
            crc = self.crc32(phy_block[0:-self.PHY_BLOCK_PBCS_WIDTH]) # calculate CRC
            self.set_bytes_field(phy_block, crc, self.PHY_BLOCK_BODY_OFFSET + body_size) # assinging CRC field
            num_segments += 1
            ssn = (ssn + 1) % 65636
            payload += phy_block

            # No more bytes, then break
            if not frames and not remainder:
                break
        stream["remainder"] = remainder
        stream["ssn"] = ssn

        return payload

    def parse_mpdu_payload(self, msdu_payload):
        phy_block_overhead_size = self.PHY_BLOCK_HEADER_WIDTH + self.PHY_BLOCK_PBCS_WIDTH
        if len(msdu_payload) > 128 + phy_block_overhead_size:
            phy_block_size = 512 + phy_block_overhead_size
        else:
            phy_block_size = 128 + phy_block_overhead_size
        j = 0
        while (j < len(msdu_payload)):
            phy_block = msdu_payload[j:j + phy_block_size]
            ssn = self.get_numeric_field(phy_block, self.PHY_BLOCK_HEADER_SSN_OFFSET, self.PHY_BLOCK_HEADER_SSN_WIDTH)
            if ssn != self.last_ssn+1:
                sys.stderr.write("MAC: discontinued SSN numbering (last = " +str(self.last_ssn) + ", current = " + str(ssn) + "\n")
            self.last_ssn = ssn
            mac_boundary_offset = self.get_numeric_field(phy_block, self.PHY_BLOCK_HEADER_MFBO_OFFSET, self.PHY_BLOCK_HEADER_MFBO_WIDTH)
            mac_boundary_flag = self.get_numeric_field(phy_block, self.PHY_BLOCK_HEADER_MMBF_OFFSET, self.PHY_BLOCK_HEADER_MMBF_WIDTH)
            segment = self.get_bytes_field(phy_block, self.PHY_BLOCK_BODY_OFFSET, phy_block_size - phy_block_overhead_size)
            crc = self.get_bytes_field(phy_block, self.PHY_BLOCK_BODY_OFFSET + len(segment), self.PHY_BLOCK_PBCS_WIDTH)
            j += phy_block_size

            if not self.crc32_check(phy_block):
                sys.stderr.write("MAC: PHY block CRC error\n")
            
            # Parsing segment
            i = 0
            if not mac_boundary_flag: mac_boundary_offset = len(segment) # if not mac boundary, use the whole segment
            mac_frame_data = segment[0:mac_boundary_offset]

            if mac_frame_data and ssn in self.rx_incomplete_frames:
                incomplete_frame = self.rx_incomplete_frames[ssn]
                incomplete_frame["data"] += mac_frame_data
                if incomplete_frame["length"] == 1: # length = 1 means the length is not calculated yet
                    header = incomplete_frame["data"][0:2]
                    incomplete_frame["length"] = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + self.MAC_FRAME_MFH_WIDTH + self.MAC_FRAME_ICV_WIDTH + 1                        
                if incomplete_frame["length"] == len(incomplete_frame["data"]):
                    self.receive_mac_frame(incomplete_frame["data"])
                else:
                    del self.rx_incomplete_frames[ssn]
                    next_ssn = (ssn + 1) % 65636
                    self.rx_incomplete_frames[next_ssn] = incomplete_frame

            i += len(mac_frame_data)
            while i < len(segment):
                length = 1
                if len(segment) - i >= self.MAC_FRAME_MFH_WIDTH:
                    header = segment[i:i + self.MAC_FRAME_MFH_WIDTH]
                    if header[0] & 0x03 == 0: # indicates zero padding segment
                        break
                    length = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + self.MAC_FRAME_MFH_WIDTH + self.MAC_FRAME_ICV_WIDTH + 1
                mac_frame_data = segment[i:i + length]
                if len(mac_frame_data) != length:
                    incomplete_frame = {}
                    incomplete_frame["data"] = mac_frame_data
                    incomplete_frame["length"] = length
                    next_ssn = (ssn + 1) % 65636
                    self.rx_incomplete_frames[next_ssn] = incomplete_frame
                else:
                    self.receive_mac_frame(mac_frame_data)
                i += len(mac_frame_data)

    def set_bytes_field(self, frame, bytes, offset):
        frame[offset:offset+len(bytes)] = bytes

    def get_bytes_field(self, frame, offset, width):
        return frame[offset:offset+width]

    def set_numeric_field(self, frame, value, offset, width = 1):
        shift = offset % 8
        shifted_length = ((shift + width - 1) / 8) + 1
        prev_byte = 0
        for i in range(shifted_length):
            byte = (value & (0xFF << i*8)) >> i*8
            frame[offset/8 + i] |= (byte << shift) & 0xFF | (prev_byte >> (8-shift))
            prev_byte = byte

    def get_numeric_field(self, frame, offset, width = 1):
        value = 0
        shift = offset % 8
        shifted_length = ((shift + width - 1) / 8) + 1
        for i in range(shifted_length):
            value |= (frame[offset/8 + i] << i*8) >> shift
        value &= ((1 << width) - 1)
        return value

    def crc32(self, data):
        crc = binascii.crc32(data)
        return bytearray([crc & 0xFF, (crc >> 8) & 0xFF, (crc >> 16) & 0xFF, (crc >> 24) & 0xFF])

    def crc32_check(self, data):
        return (binascii.crc32(data) % 0xffffffff) == 0x2144df1c
