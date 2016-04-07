#!/usr/bin/env python
import ieee1901
import binascii
import sys
from datetime import datetime, timedelta
import threading
from gnuradio import gr
from time import sleep
from transitions import MachineGraphSupport as Machine

class mac(gr.basic_block, Machine):
    MAX_SEGMENTS = 3 # max number of segments (PHY blocks) in one MAC frame
    MAX_FRAMES_IN_BUFFER = 10 # max number of MAC frames in tx buffer
    SOUND_FRAME_RATE = 0.3 # minimum time in seconds between sounds frames
    # SOUND_FRAME_RATE = 1000 # minimum time in seconds between sounds frames
    SACK_TIMEOUT = 1 # minimum time in seconds to wait for sack
    SOF_FRAME_RATE = 0.001 # minimum time in seconds between SOF frames

    rx_incomplete_frames = {}
    rx_incomplete_mgmt_frames = {}
    tx_frames_queue = {}
    tx_frames_in_queue = 0
    last_rx_ssn = -1 # track last ssn received
    last_rx_frame_blocks_error = [] # list of last received frame blocks error
    last_rx_frame_type = ""
    last_tx_sound_frame = datetime.min # last time a sound frame was transmitted
    last_tx_frame_n_blocks = 0 # number of PHY blocks in last sent frame
    last_tx_n_errors = 0 # number of error PHY blocks in the last frame sent
    rx_tone_map = []
    tx_tone_map = []
    tx_capacity = 0
    transmission_queue_is_full = False
    sack_timer = None
    sof_timer = None
    stats = {'n_blocks_tx_success': 0, 'n_blocks_tx_fail': 0, 'n_missing_acks': 0}

    def __init__(self, device_addr, master, tmi, dest, broadcast_tone_mask, sync_tone_mask, force_tone_mask, target_ber, channel_est_mode, info, debug):
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
        self.dest = bytearray(''.join(chr(x) for x in dest))
        self.debug = debug
        self.info = info
        self.is_master = master
        self.tmi = tmi
        self.broadcast_tone_mask = broadcast_tone_mask
        self.sync_tone_mask = sync_tone_mask
        self.force_tone_mask = force_tone_mask
        self.target_ber = target_ber
        self.channel_est_mode = channel_est_mode
        if self.is_master:
            self.name = "MAC (master)"
            initial_state = 'waiting_for_app'
        else:
            self.name = "MAC (slave)"
            initial_state = 'waiting_for_sof_sound'

        states = [
            # Master:
            {'name': 'waiting_for_app'                                                                                  },
            {'name': 'sending_sof'              ,'on_enter': ['wait_for_sof_timer', 'transmit_sof', 'start_sof_timer']  },
            {'name': 'sending_sound'            ,'on_enter': 'transmit_sound'                                           },
            {'name': 'waiting_for_sack'         ,'on_enter': 'start_sack_timer', 'on_exit': 'cancel_sack_timer'         },
            {'name': 'waiting_for_soundack'                                                                             },
            {'name': 'waiting_for_mgmtmsg'                                                                              },

            # Slave:
            {'name': 'sending_sack'             ,'on_enter': 'transmit_sack'                                            },
            {'name': 'sending_soundack'         ,'on_enter': 'transmit_sack'                                            },
            {'name': 'sending_mgmtmsg'          ,'on_enter': 'transmit_mgmtmsg'                                         },
            {'name': 'waiting_for_sof_sound'                                                                            },
            {'name': 'waiting_for_tone_map'     ,'on_enter': 'send_calc_tone_info_to_phy'                               },
        ]
        transitions = [
            # TRIGGER                   SOURCE                      DESTINATION                 CONDITIONS          UNLESS                                  BEFORE  AFTER

            # Master:
            ['event_msdu_arrived'       ,'waiting_for_app'          ,'sending_sof'              ,None               ,'sound_timeout'                        ,None               ,None                   ],
            ['event_msdu_arrived'       ,'waiting_for_app'          ,'sending_sound'            ,'sound_timeout'    ,None                                   ,None               ,None                   ],
            ['event_tx_end'             ,'sending_sof'              ,'waiting_for_sack'         ,None               ,None                                   ,None               ,None                   ],
            ['event_tx_end'             ,'sending_sound'            ,'waiting_for_soundack'     ,None               ,None                                   ,None               ,None                   ],
            ['event_sack_arrived'       ,'waiting_for_sack'         ,'sending_sof'              ,None               ,['queue_is_empty', 'sound_timeout']    ,None               ,'update_blocks_stats'  ],
            ['event_sack_arrived'       ,'waiting_for_sack'         ,'sending_sound'            ,'sound_timeout'    ,'queue_is_empty'                       ,None               ,'update_blocks_stats'  ],
            ['event_sack_arrived'       ,'waiting_for_sack'         ,'waiting_for_app'          ,'queue_is_empty'   ,None                                   ,None               ,'update_blocks_stats'  ],
            ['event_sack_timout'        ,'waiting_for_sack'         ,'sending_sof'              ,None               ,['queue_is_empty', 'sound_timeout']    ,None               ,None                   ],
            ['event_sack_timout'        ,'waiting_for_sack'         ,'sending_sound'            ,'sound_timeout'    ,'queue_is_empty'                       ,None               ,None                   ],
            ['event_sack_timeout'       ,'waiting_for_sack'         ,'waiting_for_app'          ,'queue_is_empty'   ,None                                   ,None               ,None                   ],
            ['event_sack_arrived'       ,'waiting_for_soundack'     ,'waiting_for_mgmtmsg'      ,None               ,None                                   ,None               ,None                   ],
            ['event_sof_arrived'        ,'waiting_for_mgmtmsg'      ,'sending_sof'              ,None               ,None                                   ,None               ,None                   ],

            # Slave:
            ['event_tx_end'             ,'sending_sack'             ,'waiting_for_sof_sound'    ,None               ,None                                   ,None               ,None                   ],
            ['event_tx_end'             ,'sending_soundack'         ,'sending_mgmtmsg'          ,None               ,None                                   ,None               ,None                   ],
            ['event_tx_end'             ,'sending_mgmtmsg'          ,'waiting_for_sof_sound'    ,None               ,None                                   ,None               ,None                   ],
            ['event_sof_arrived'        ,'waiting_for_sof_sound'    ,'sending_sack'             ,None               ,None                                   ,'post_process_payload'  ,None                   ],
            ['event_sound_arrived'      ,'waiting_for_sof_sound'    ,'waiting_for_tone_map'     ,None               ,None                                   ,'post_process_payload'  ,None                   ],
            ['event_tone_map_arrived'   ,'waiting_for_tone_map'     ,'sending_soundack'         ,None               ,None                                   ,None               ,None                   ],
        ]

        Machine.__init__(self, states=states, transitions=transitions, auto_transitions=False, initial=initial_state)
        if self.debug:
            graph = self.get_graph()
            graph.draw(self.name + '.png', prog='dot')

    def capacity(self):
        return self.tx_capacity

    def app_in_handler(self, msg):
        if gr.pmt.is_pair(msg):
            cdr = gr.pmt.cdr(msg)
            car = gr.pmt.car(msg)
            if gr.pmt.is_symbol(car) and gr.pmt.is_dict(cdr):
                msg_id = gr.pmt.to_python(car)
                dict = gr.pmt.to_python(cdr)
                if msg_id == "MAC-TXMSDU":
                    payload = bytearray(dict["msdu"])
                    if self.debug: print self.name + ": state = " + str(self.state) + ", received payload from APP, size = " + str(len(payload)) + ", frames in buffer = " + str(self.tx_frames_in_queue)
                    if self.tx_frames_in_queue < self.MAX_FRAMES_IN_BUFFER:
                        mac_frame = self.create_mac_frame(self.dest, payload, False)
                        self.submit_mac_frame(mac_frame)
                        if not self.transmission_queue_is_full:
                            self.send_status_to_app()
                        if self.state == 'waiting_for_app': self.event_msdu_arrived()
                    else :
                        sys.stderr.write (self.name + ": state = " + str(self.state) + ", buffer is full, dropping frame from APP\n")
                        if self.debug: print self.name + ": state = " + str(self.state) + ", buffer is full, dropping frame from APP"

    def phy_in_handler(self, msg):
        if gr.pmt.is_pair(msg):
            cdr = gr.pmt.cdr(msg)
            car = gr.pmt.car(msg)
            if gr.pmt.is_symbol(car) and gr.pmt.is_dict(cdr):
                msg_id = gr.pmt.to_python(car)
                dict = gr.pmt.to_python(cdr)
                if msg_id == "PHY-RXSTART":
                    frame_control = bytearray(dict["frame_control"])
                    dt = self.get_frame_type(frame_control)
                    self.last_rx_frame_type = dt
                    self.last_rx_frame_blocks_error = []
                    if dt == "SOF":
                        if self.debug: print self.name + ": state = " + str(self.state) + ", received MPDU (SOF) from PHY"
                        payload = bytearray(dict["payload"].tolist())
                        self.last_rx_frame_blocks_error = self.parse_mpdu_payload(payload)
                    elif dt == "SOUND":
                        if self.debug: print self.name + ": state = " + str(self.state) + ", received MPDU (Sound) from PHY"
                    elif dt == "SACK":
                        if self.debug: print self.name + ": state = " + str(self.state) + ", received MPDU (SACK) from PHY"
                        sackd = self.get_bytes_field(frame_control, ieee1901.FRAME_CONTROL_SACK_SACKD_OFFSET/8, ieee1901.FRAME_CONTROL_SACK_SACKD_WIDTH/8)
                        self.last_tx_n_errors = self.parse_sackd(sackd)
                        if (self.last_tx_n_errors): sys.stderr.write(self.name + ": state = " + str(self.state) + ", SACK indicates " + str(self.last_tx_n_errors) + " blocks error\n")

                if msg_id == "PHY-RXEND":
                    if self.debug: print self.name + ": state = " + str(self.state) + ", received PHY-RXEND from PHY"
                    if self.last_rx_frame_type == "SACK":
                        self.event_sack_arrived()
                    elif self.last_rx_frame_type == "SOF":
                        self.event_sof_arrived()
                    elif self.last_rx_frame_type == "SOUND":
                        self.event_sound_arrived()

                elif msg_id == "PHY-TXEND":
                    if self.debug: print self.name + ": state = " + str(self.state) + ", received PHY-TXEND from PHY"
                    self.event_tx_end()

                elif msg_id == "PHY-RXCALCTONEMAP.response":
                    if self.debug: print self.name + ": state = " + str(self.state) + ", received PHY-RXCALCTONEMAP.respond from PHY"
                    self.rx_tone_map = bytearray(dict["tone_map"].tolist())
                    self.event_tone_map_arrived()

    def sack_timout_callback(self):
        sys.stderr.write(self.name + ": state = " + str(self.state) + ", Error: SACK timeout\n")
        self.stats['n_missing_acks'] += 1
        self.event_sack_timout()

    def start_sack_timer(self):
        self.sack_timer = threading.Timer(self.SACK_TIMEOUT, self.sack_timout_callback)
        self.sack_timer.start()

    def cancel_sack_timer(self):
        if self.sack_timer:
            self.sack_timer.cancel()
            self.sack_time = None

    def start_sof_timer(self):
        self.sof_timer = threading.Timer(self.SOF_FRAME_RATE, self.sof_timeout_callback)
        self.sof_timer.start()

    def sof_timeout_callback(self):
        pass

    def wait_for_sof_timer(self):
        if self.sof_timer:
            self.sof_timer.join()
            self.sof_timer = None

    def update_blocks_stats(self):
        self.stats['n_blocks_tx_success'] += self.last_tx_frame_n_blocks - self.last_tx_n_errors
        self.stats['n_blocks_tx_fail'] += self.last_tx_n_errors

    def transmit_sof(self):
        mpdu_payload, self.last_tx_frame_n_blocks = self.create_mpdu_payload(False)
        mpdu_payload_pmt = gr.pmt.init_u8vector(len(mpdu_payload), list(mpdu_payload))

        # Create frame control
        mpdu_fc = self.create_sof_frame_control(self.tmi, mpdu_payload)
        mpdu_fc_pmt = gr.pmt.init_u8vector(len(mpdu_fc), list(mpdu_fc))

        # Send the SOF frame to PHY
        dict = gr.pmt.make_dict();
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("frame_control"), mpdu_fc_pmt)
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("payload"), mpdu_payload_pmt)
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending MPDU (SOF, tmi=" + str(self.tmi) + ") to PHY"
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXSTART"), dict))

    def transmit_sound(self):
        # Create zeros payload
        mpdu_payload = bytearray(520)
        mpdu_payload_pmt = gr.pmt.init_u8vector(len(mpdu_payload), list(mpdu_payload))

        # Create frame control
        mpdu_fc = self.create_sound_frame_control("PB520")
        mpdu_fc_pmt = gr.pmt.init_u8vector(len(mpdu_fc), list(mpdu_fc))

        # Send SOUND frame to PHY
        dict = gr.pmt.make_dict();
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("frame_control"), mpdu_fc_pmt)
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("payload"), mpdu_payload_pmt)
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending MPDU (Sound) to PHY"
        self.last_tx_sound_frame = datetime.now();
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXSTART"), dict))

    def transmit_sack(self):
        # Preparing SACK frame control
        sackd = bytearray(((len(self.last_rx_frame_blocks_error) - 1) / 8) + 1 + 1)
        sackd[0] = 0b00010101
        for i in range(len(self.last_rx_frame_blocks_error)):
            sackd[1 + i/8] = sackd[1 + i/8] | (self.last_rx_frame_blocks_error[i] << (i % 8))
        sackd_u8vector = gr.pmt.init_u8vector(len(sackd), list(sackd))
        mpdu_fc = self.create_sack_frame_control(sackd)
        mpdu_fc_pmt = gr.pmt.init_u8vector(len(mpdu_fc), list(mpdu_fc))

        dict = gr.pmt.make_dict()
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("frame_control"), mpdu_fc_pmt)
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending MPDU (SACK) to PHY"
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXSTART"), dict))

    def transmit_mgmtmsg(self):
        # Creating the management MAC frame
        mpdu_payload, self.last_tx_frame_n_blocks = self.create_mpdu_payload(self.create_mgmt_msg_cm_chan_est(self.rx_tone_map))

        # Create frame control
        mpdu_fc = self.create_sof_frame_control(1, mpdu_payload)
        mpdu_fc_pmt = gr.pmt.init_u8vector(len(mpdu_fc), list(mpdu_fc))
        mpdu_payload_pmt = gr.pmt.init_u8vector(len(mpdu_payload), list(mpdu_payload))

        # Sending the frame to PHY
        dict = gr.pmt.make_dict();
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("frame_control"), mpdu_fc_pmt)
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("payload"), mpdu_payload_pmt)
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending MPDU (SOF MGMT, tmi=1) to PHY"
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXSTART"), dict))

    def send_calc_tone_info_to_phy(self):
        dict = gr.pmt.make_dict();
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("target_ber"), gr.pmt.to_pmt(self.target_ber))
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-RXCALCTONEMAP.request"), dict))
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending PHY-RXCALCTONEMAP.request"

    def send_set_tx_tone_map(self):
        tone_map_pmt = gr.pmt.init_u8vector(len(self.tx_tone_map), list(self.tx_tone_map))
        dict = gr.pmt.make_dict()
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("tone_map"), tone_map_pmt)
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXCONFIG"), dict))
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending PHY-TXCONFIG"

    def send_init_phy(self):
        tone_mask_pmt = gr.pmt.init_u8vector(len(self.broadcast_tone_mask), list(self.broadcast_tone_mask))
        sync_tone_mask_pmt = gr.pmt.init_u8vector(len(self.sync_tone_mask), list(self.sync_tone_mask))
        dict = gr.pmt.make_dict()
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("broadcast_tone_mask"), tone_mask_pmt)
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("sync_tone_mask"), sync_tone_mask_pmt)
        dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("channel_est_mode"), gr.pmt.to_pmt(self.channel_est_mode))
        if (self.force_tone_mask):
            force_tone_mask_pmt = gr.pmt.init_u8vector(len(self.force_tone_mask), list(self.force_tone_mask))
            dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("force_tone_mask"), force_tone_mask_pmt)

        if self.is_master:
            dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("id"), gr.pmt.to_pmt("master"))
        else:
            dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("id"), gr.pmt.to_pmt("slave"))

        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-RXINIT"), dict))
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending PHY-RXINIT"
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-TXINIT"), dict))
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending PHY-TXINIT"

    def send_post_process_payload_to_phy(self):
        dict = gr.pmt.make_dict();
        self.message_port_pub(gr.pmt.to_pmt("phy out"), gr.pmt.cons(gr.pmt.to_pmt("PHY-RXPOSTPROCESSPAYLOAD"), dict))
        if self.debug: print self.name + ": state = " + str(self.state) + ", sending PHY-RXPOSTPROCESSPAYLOAD"

    def send_status_to_app(self):
        self.message_port_pub(gr.pmt.to_pmt("app out"), gr.pmt.cons(gr.pmt.to_pmt("MAC-READY"), gr.pmt.PMT_NIL))

    def queue_is_empty(self):
        return self.tx_frames_in_queue == 0

    def sound_timeout(self):
        return (datetime.now() - self.last_tx_sound_frame).total_seconds() >= self.SOUND_FRAME_RATE

    def post_process_payload(self):
        if not (1 in self.last_rx_frame_blocks_error): self.send_post_process_payload_to_phy()

    def forecast(self, noutput_items, ninput_items_required):
        #setup size of input_items[i] for work call
        for i in range(len(ninput_items_required)):
            ninput_items_required[i] = noutput_items

    def start(self):
        self.send_init_phy();
        self.send_status_to_app()

    def stop(self):
        self.cancel_sack_timer()
        if self.is_master:
            print self.name + " final report:"
            print "PHY blocks transmitted successfully: " + str(self.stats['n_blocks_tx_success'])
            print "PHY blocks transmitted unsuccessfully: " + str(self.stats['n_blocks_tx_fail'])
            print "Missing SACK frames: " + str(self.stats['n_missing_acks'])
        return True

    def general_work(self, input_items, output_items):
        noutput_items = output_items[0].size
        ninput_items = input_items[0].size
        n = min(noutput_items, ninput_items)
        self.consume(0, len(input_items[0]))
        return n

    def create_mac_frame(self, dest, payload, mgmt):
        if (not mgmt):
            mft = 0b01 # MSDU payload
            mac_frame = bytearray(ieee1901.MAC_FRAME_OVERHEAD + len(payload)) # preallocate the frame
        else:
            mft = 0b11 # management payload
            mac_frame = bytearray(ieee1901.MAC_FRAME_OVERHEAD + ieee1901.MAC_FRAME_CONFOUNDER_WIDTH + len(payload)) # preallocate the frame

        mfl = len(mac_frame) - ieee1901.MAC_FRAME_MFH_WIDTH - ieee1901.MAC_FRAME_ICV_WIDTH - 1 # calculate frame length field
        header = bytearray(2) # header is 2 bytes
        header[0] = (mfl & 0x3F) << 2 | mft  # concat frame legnth (14 bits) with frame type (2 bits)
        header[1] = (mfl & 0x3FC0) >> 6
        pos = self.set_bytes_field(mac_frame, header, ieee1901.MAC_FRAME_MFH_OFFSET, ieee1901.MAC_FRAME_MFH_WIDTH)
        if mgmt:  # only mgmt has confounder field
            self.set_numeric_field(mac_frame, 0, pos*8, ieee1901.MAC_FRAME_CONFOUNDER_WIDTH*8)
            pos += ieee1901.MAC_FRAME_CONFOUNDER_WIDTH
        pos = self.set_bytes_field(mac_frame, dest, pos, ieee1901.MAC_FRAME_ODA_WIDTH)
        pos = self.set_bytes_field(mac_frame, self.device_addr, pos, ieee1901.MAC_FRAME_OSA_WIDTH)
        pos += ieee1901.MAC_FRAME_ETHERTYPE_OR_LENGTH_WIDTH
        pos = self.set_bytes_field(mac_frame, payload, pos, len(payload))
        crc = self.crc32(mac_frame[ieee1901.MAC_FRAME_MFH_OFFSET + ieee1901.MAC_FRAME_MFH_WIDTH:-ieee1901.MAC_FRAME_ICV_WIDTH]) # calculate crc excluding MFH and ICV fields
        self.set_bytes_field(mac_frame, crc, pos, ieee1901.MAC_FRAME_ICV_WIDTH)

        return mac_frame

    def parse_mac_frame(self, mac_frame):
        pos = ieee1901.MAC_FRAME_MFH_OFFSET
        header = self.get_bytes_field(mac_frame, pos, ieee1901.MAC_FRAME_MFH_WIDTH)
        pos += ieee1901.MAC_FRAME_MFH_WIDTH
        mft = header[0] & 0x3
        length = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + ieee1901.MAC_FRAME_MFH_WIDTH + ieee1901.MAC_FRAME_ICV_WIDTH + 1
        if mft == 0b11: # determine frame type (management or data)
            confounder = self.get_bytes_field(mac_frame, pos, ieee1901.MAC_FRAME_CONFOUNDER_WIDTH)
            pos += ieee1901.MAC_FRAME_CONFOUNDER_WIDTH
            mgmt = True
        elif mft == 0b01:
            mgmt = False
        else:
            sys.stderr.write(self.name + ": state = " + str(self.state) + ", MAC frame type not supported\n")
            return

        dest_addr = self.get_bytes_field(mac_frame, pos, ieee1901.MAC_FRAME_ODA_WIDTH)
        pos += ieee1901.MAC_FRAME_ODA_WIDTH
        source_addr = self.get_bytes_field(mac_frame, pos, ieee1901.MAC_FRAME_OSA_WIDTH)
        pos += ieee1901.MAC_FRAME_OSA_WIDTH + ieee1901.MAC_FRAME_ETHERTYPE_OR_LENGTH_WIDTH
        payload = self.get_bytes_field(mac_frame, pos, length - pos - ieee1901.MAC_FRAME_ICV_WIDTH)
        if not self.crc32_check(mac_frame[ieee1901.MAC_FRAME_MFH_OFFSET + ieee1901.MAC_FRAME_MFH_WIDTH:]):
            sys.stderr.write(self.name + ": state = " + str(self.state) + ", MAC frame CRC error\n")
        return (payload, mgmt)

    def submit_mac_frame(self, frame):
        if (str(self.dest) in self.tx_frames_queue):
            stream = self.tx_frames_queue[str(self.dest)]
        else:
            stream = {"ssn": 0, "frames": [], "remainder": bytearray(0)}
            self.tx_frames_queue[str(self.dest)] = stream
        stream["frames"].append(frame)
        self.tx_frames_in_queue += 1
        if self.tx_frames_in_queue == self.MAX_FRAMES_IN_BUFFER: self.transmission_queue_is_full = True
        if self.debug: print self.name + ": state = " + str(self.state) + ", added MAC frame to tranmission queue, frame size = " + str(len(frame))

    def receive_mac_frame(self, frame):
        payload, mgmt = self.parse_mac_frame(frame)
        if payload:
            if not mgmt:
                if self.debug: print self.name + ": state = " + str(self.state) + ", received MAC frame, size = " + str(len(frame)) + ", sending payload to APP"
                payload_u8vector = gr.pmt.init_u8vector(len(payload), list(payload))
                dict = gr.pmt.make_dict();
                dict = gr.pmt.dict_add(dict, gr.pmt.to_pmt("msdu"), payload_u8vector)
                self.message_port_pub(gr.pmt.to_pmt("app out"), gr.pmt.cons(gr.pmt.to_pmt("MAC-RXMSDU"), dict));
            else:
                if self.debug: print self.name + ": state = " + str(self.state) + ", received MAC frame (management), size = " + str(len(frame))
                self.process_mgmt_msg(payload)

    def init_phy_block(self, size, ssn, mac_boundary_offset, valid_flag, message_queue_flag, mac_boundary_flag, oldest_ssn_flag):
        phy_block = bytearray(size + 8)
        self.set_numeric_field(phy_block, ssn, ieee1901.PHY_BLOCK_HEADER_SSN_OFFSET, ieee1901.PHY_BLOCK_HEADER_SSN_WIDTH)
        if mac_boundary_flag == 1:
            self.set_numeric_field(phy_block, mac_boundary_offset, ieee1901.PHY_BLOCK_HEADER_MFBO_OFFSET, ieee1901.PHY_BLOCK_HEADER_MFBO_WIDTH)
        self.set_numeric_field(phy_block, valid_flag, ieee1901.PHY_BLOCK_HEADER_VPBF_OFFSET, ieee1901.PHY_BLOCK_HEADER_VPBF_WIDTH)
        self.set_numeric_field(phy_block, message_queue_flag, ieee1901.PHY_BLOCK_HEADER_MMQF_OFFSET, ieee1901.PHY_BLOCK_HEADER_MMQF_WIDTH)
        self.set_numeric_field(phy_block, mac_boundary_flag, ieee1901.PHY_BLOCK_HEADER_MMBF_OFFSET, ieee1901.PHY_BLOCK_HEADER_MMBF_WIDTH)
        self.set_numeric_field(phy_block, oldest_ssn_flag, ieee1901.PHY_BLOCK_HEADER_OPSF_OFFSET, ieee1901.PHY_BLOCK_HEADER_OPSF_WIDTH)
        return phy_block

    def create_mpdu_payload(self, mgmt = []):  # if mgmt is empty, get frames from tx queue, else, create mpdu payload using the mgmt msg
        stream = {}
        if not mgmt:
            # Check if buffer is empty
            if not len(self.tx_frames_queue):
                return ([], 0)
            for key in self.tx_frames_queue:
                if self.tx_frames_queue[key]["frames"] or self.tx_frames_queue[key]["remainder"]:
                    stream = self.tx_frames_queue[key]
                    break
            if not stream:
                return ([], 0)
        else:
            stream["frames"] = [mgmt]
            stream["remainder"] = []
            stream["ssn"] = 0

        frames = stream["frames"]
        ssn = 0
        remainder = stream["remainder"]
        num_segments = 0
        payload = bytearray(0)

        while num_segments < self.MAX_SEGMENTS or mgmt:
            # Creating a new segment
            mac_boundary_flag = False
            mac_boundary_offset = 0
            body_size = 512
            segment = remainder[0:body_size]
            if remainder:
                remainder = remainder[body_size:]
                if not remainder: self.tx_frames_in_queue -= 1
            while len(segment) < body_size and frames:
                if not mac_boundary_flag:
                    mac_boundary_offset = len(segment)
                    mac_boundary_flag = True
                remainder = frames[0][body_size-len(segment):]
                segment += frames[0][0:body_size-len(segment)]
                frames.pop(0)
                if not remainder: self.tx_frames_in_queue -= 1

            # Determine if segment should be 128 bytes or 512
            if len(segment) < body_size:
                if len(segment) < 128 and num_segments == 0: body_size = 128
                if not mac_boundary_flag:
                    mac_boundary_offset = len(segment)
                    mac_boundary_flag = True
                segment[len(segment):body_size] = bytearray(body_size-len(segment))

            # Creating the PHY block
            phy_block = self.init_phy_block(body_size, ssn, mac_boundary_offset, True, not mgmt==[], mac_boundary_flag, False) # Setting header fields
            pos = self.set_bytes_field(phy_block, segment, ieee1901.PHY_BLOCK_BODY_OFFSET, body_size) # assigning body
            crc = self.crc32(phy_block[0:-ieee1901.PHY_BLOCK_PBCS_WIDTH]) # calculate CRC
            self.set_bytes_field(phy_block, crc, pos, ieee1901.PHY_BLOCK_PBCS_WIDTH) # assinging CRC field
            num_segments += 1
            ssn = (ssn + 1) % 65636
            payload += phy_block

            # No more bytes, then break
            if not frames and not remainder:
                break

        if not mgmt:
            stream["remainder"] = remainder
            stream["ssn"] = ssn
            if self.transmission_queue_is_full and self.tx_frames_in_queue < self.MAX_FRAMES_IN_BUFFER:
                self.transmission_queue_is_full = False
                self.send_status_to_app()

        return (payload, num_segments)

    def parse_mpdu_payload(self, mpdu_payload):
        phy_blocks_error = []
        phy_block_overhead_size = ieee1901.PHY_BLOCK_HEADER_WIDTH + ieee1901.PHY_BLOCK_PBCS_WIDTH
        if len(mpdu_payload) > 128 + phy_block_overhead_size:
            phy_block_size = 512 + phy_block_overhead_size
        else:
            phy_block_size = 128 + phy_block_overhead_size
        j = 0
        prev_mgmt_ssn = -1
        while (j < len(mpdu_payload)):
            phy_block = mpdu_payload[j:j + phy_block_size]
            ssn = self.get_numeric_field(phy_block, ieee1901.PHY_BLOCK_HEADER_SSN_OFFSET, ieee1901.PHY_BLOCK_HEADER_SSN_WIDTH)
            mgmt_queue = self.get_numeric_field(phy_block, ieee1901.PHY_BLOCK_HEADER_MMQF_OFFSET, ieee1901.PHY_BLOCK_HEADER_MMQF_WIDTH)
            if (mgmt_queue):
                if ssn != prev_mgmt_ssn + 1:
                    sys.stderr.write(self.name + ": state = " + str(self.state) + ", discontinued MGMT SSN numbering (last = " +str(prev_mgmt_ssn) + ", current = " + str(ssn) + "\n")
                prev_mgmt_ssn = ssn
                incomplete_frames = self.rx_incomplete_mgmt_frames

            else:
                if ssn != self.last_rx_ssn + 1:
                    sys.stderr.write(self.name + ": state = " + str(self.state) + ", discontinued SSN numbering (last = " +str(self.last_rx_ssn) + ", current = " + str(ssn) + "\n")
                self.last_rx_ssn = ssn
                incomplete_frames = self.rx_incomplete_frames

            mac_boundary_offset = self.get_numeric_field(phy_block, ieee1901.PHY_BLOCK_HEADER_MFBO_OFFSET, ieee1901.PHY_BLOCK_HEADER_MFBO_WIDTH)
            mac_boundary_flag = self.get_numeric_field(phy_block, ieee1901.PHY_BLOCK_HEADER_MMBF_OFFSET, ieee1901.PHY_BLOCK_HEADER_MMBF_WIDTH)
            segment = self.get_bytes_field(phy_block, ieee1901.PHY_BLOCK_BODY_OFFSET, phy_block_size - phy_block_overhead_size)
            crc = self.get_bytes_field(phy_block, ieee1901.PHY_BLOCK_BODY_OFFSET + len(segment), ieee1901.PHY_BLOCK_PBCS_WIDTH)
            j += phy_block_size

            if not self.crc32_check(phy_block):
                sys.stderr.write(self.name + ": state = " + str(self.state) + ", PHY block CRC error\n")
                phy_blocks_error.append(1)
                continue
            else:
                phy_blocks_error.append(0)

            # Parsing segment
            i = 0
            if not mac_boundary_flag: mac_boundary_offset = len(segment) # if not mac boundary, use the whole segment
            mac_frame_data = segment[0:mac_boundary_offset]

            if mac_frame_data and ssn in incomplete_frames:
                incomplete_frame = incomplete_frames[ssn]
                incomplete_frame["data"] += mac_frame_data
                if incomplete_frame["length"] == 1: # length = 1 means the length is not calculated yet
                    header = incomplete_frame["data"][0:2]
                    incomplete_frame["length"] = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + ieee1901.MAC_FRAME_MFH_WIDTH + ieee1901.MAC_FRAME_ICV_WIDTH + 1
                if incomplete_frame["length"] == len(incomplete_frame["data"]):
                    self.receive_mac_frame(incomplete_frame["data"])
                else:
                    del incomplete_frames[ssn]
                    next_ssn = (ssn + 1) % 65636
                    incomplete_frames[next_ssn] = incomplete_frame

            i += len(mac_frame_data)
            while i < len(segment):
                length = 1
                if len(segment) - i >= ieee1901.MAC_FRAME_MFH_WIDTH:
                    header = segment[i:i + ieee1901.MAC_FRAME_MFH_WIDTH]
                    if header[0] & 0x03 == 0: # indicates zero padding segment
                        break
                    length = (((header[0] & 0xFC) >> 2) | (header[1] << 6)) + ieee1901.MAC_FRAME_MFH_WIDTH + ieee1901.MAC_FRAME_ICV_WIDTH + 1
                mac_frame_data = segment[i:i + length]
                if len(mac_frame_data) != length:
                    incomplete_frame = {}
                    incomplete_frame["data"] = mac_frame_data
                    incomplete_frame["length"] = length
                    next_ssn = (ssn + 1) % 65636
                    incomplete_frames[next_ssn] = incomplete_frame
                else:
                    self.receive_mac_frame(mac_frame_data)
                i += len(mac_frame_data)

        return phy_blocks_error

    def parse_sackd(self, sackd):
        sacki = sackd[0]
        if not sacki == 0b00010101:
            sys.stderr.write(self.name + ": state = " + str(self.state) + ", not supported sacki = " + str(sacki) + "\n")
        n_errors = 0
        for i in range(self.last_tx_frame_n_blocks):
            n_errors += not (sackd[1 + i/8] & (1 << (i % 8)) == 0)
        return n_errors

    def create_mgmt_msg_cm_chan_est(self, tone_map):
        mmentry = bytearray((ieee1901.MGMT_CM_CHAN_EST_NTMI_OFFSET + # preallocate the mmentry bytearray
                            ieee1901.MGMT_CM_CHAN_EST_NTMI_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_TMI_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_NINT_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_NEW_TMI_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_CPF_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_FECTYPE_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_GIL_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_CBD_ENC_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_CBD_LEN_WIDTH +
                            ieee1901.MGMT_CM_CHAN_EST_CBD_WIDTH * (len(tone_map)+1))/8)
        pos = self.set_numeric_field(mmentry, 1, ieee1901.MGMT_CM_CHAN_EST_NTMI_OFFSET, ieee1901.MGMT_CM_CHAN_EST_NTMI_WIDTH)
        pos = self.set_numeric_field(mmentry, 1, pos, ieee1901.MGMT_CM_CHAN_EST_TMI_WIDTH)
        pos = self.set_numeric_field(mmentry, 0, pos, ieee1901.MGMT_CM_CHAN_EST_NINT_WIDTH)
        pos = self.set_numeric_field(mmentry, 1, pos, ieee1901.MGMT_CM_CHAN_EST_NEW_TMI_WIDTH)
        pos += ieee1901.MGMT_CM_CHAN_EST_CPF_WIDTH + ieee1901.MGMT_CM_CHAN_EST_FECTYPE_WIDTH + ieee1901.MGMT_CM_CHAN_EST_GIL_WIDTH
        pos = self.set_numeric_field(mmentry, 0, pos, ieee1901.MGMT_CM_CHAN_EST_CBD_ENC_WIDTH)
        pos = self.set_numeric_field(mmentry, len(tone_map), pos, ieee1901.MGMT_CM_CHAN_EST_CBD_LEN_WIDTH)
        for i in range(len(tone_map)):
            pos = self.set_numeric_field(mmentry, tone_map[i], pos, ieee1901.MGMT_CM_CHAN_EST_CBD_WIDTH)
        mgmt_msg = self.create_mgmt_msg(ieee1901.MGMT_MMTYPE_CM_CHAN_EST_ID, mmentry)
        mac_frame = self.create_mac_frame(self.dest, mgmt_msg, True)
        return mac_frame

    def process_mgmt_msg_cm_chan_est(self, mmentry):
        new_tmi_offset = ieee1901.MGMT_CM_CHAN_EST_NTMI_OFFSET + \
                         ieee1901.MGMT_CM_CHAN_EST_NTMI_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_TMI_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_NINT_WIDTH
        new_tmi = self.get_numeric_field(mmentry, new_tmi_offset, ieee1901.MGMT_CM_CHAN_EST_NEW_TMI_WIDTH)
        cbd_len_offset = new_tmi_offset + \
                         ieee1901.MGMT_CM_CHAN_EST_NEW_TMI_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_CPF_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_FECTYPE_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_GIL_WIDTH + \
                         ieee1901.MGMT_CM_CHAN_EST_CBD_ENC_WIDTH
        cbd_len = self.get_numeric_field(mmentry, cbd_len_offset, ieee1901.MGMT_CM_CHAN_EST_CBD_LEN_WIDTH)
        self.tx_tone_map = [0] * cbd_len
        cbd_offset = cbd_len_offset + ieee1901.MGMT_CM_CHAN_EST_CBD_LEN_WIDTH
        self.tx_capacity = 0
        for i in range(cbd_len):
            self.tx_tone_map[i] = self.get_numeric_field(mmentry, cbd_offset, ieee1901.MGMT_CM_CHAN_EST_CBD_WIDTH)
            self.tx_capacity += ieee1901.BITLOADING_NBITS[self.tx_tone_map[i]]
            cbd_offset += ieee1901.MGMT_CM_CHAN_EST_CBD_WIDTH
        if self.debug: print self.name + ": state = " + str(self.state) + ", TX custom tone map capacity: " + str(self.tx_capacity)
        self.send_set_tx_tone_map()
        if self.info:
            print "'" + self.name + "'; txToneMap = " + str(self.tx_tone_map) + ";"
            print "'" + self.name + "'; txCapacity = " + str(self.tx_capacity) + ";"

    def create_mgmt_msg(self, mmtype, mmentry):
        mgmt_msg = bytearray(len(mmentry) + (ieee1901.MGMT_MMV_WIDTH + ieee1901.MGMT_MMTYPE_WIDTH + ieee1901.MGMT_FMI_WIDTH)/8)
        self.set_numeric_field(mgmt_msg, 0x1, ieee1901.MGMT_MMV_OFFSET, ieee1901.MGMT_MMV_WIDTH)
        self.set_numeric_field(mgmt_msg, mmtype, ieee1901.MGMT_MMTYPE_OFFSET, ieee1901.MGMT_MMTYPE_WIDTH)
        self.set_bytes_field(mgmt_msg, mmentry, ieee1901.MGMT_MMENTRY_OFFSET/8, len(mmentry))
        return mgmt_msg

    def process_mgmt_msg(self, mgmt_msg):
        mmtype = self.get_numeric_field(mgmt_msg, ieee1901.MGMT_MMTYPE_OFFSET, ieee1901.MGMT_MMTYPE_WIDTH)
        mmentry = self.get_bytes_field(mgmt_msg, ieee1901.MGMT_MMENTRY_OFFSET/8, len(mgmt_msg) - ieee1901.MGMT_MMENTRY_OFFSET/8)
        if mmtype == ieee1901.MGMT_MMTYPE_CM_CHAN_EST_ID:
            self.process_mgmt_msg_cm_chan_est(mmentry)
        else:
            sys.stderr.write (self.name + ": state = " + str(self.state) + ", management message (" + str(mmtype) + "not supported\n")

    def create_sof_frame_control(self, tmi, mpdu_payload):
        frame_control = bytearray(ieee1901.FRAME_CONTROL_NBITS/8);

        # Set the pbsz bit
        if len(mpdu_payload) > 136/8:
            self.set_numeric_field(frame_control, 0, ieee1901.FRAME_CONTROL_SOF_PBSZ_OFFSET, ieee1901.FRAME_CONTROL_SOF_PBSZ_WIDTH)
        else:
            self.set_numeric_field(frame_control, 1, ieee1901.FRAME_CONTROL_SOF_PBSZ_OFFSET, ieee1901.FRAME_CONTROL_SOF_PBSZ_WIDTH)

        # Set delimiter type to SOF
        self.set_numeric_field(frame_control, 1, ieee1901.FRAME_CONTROL_DT_IH_OFFSET, ieee1901.FRAME_CONTROL_DT_IH_WIDTH)

        # Set tone map index
        self.set_numeric_field(frame_control, tmi, ieee1901.FRAME_CONTROL_SOF_TMI_OFFSET, ieee1901.FRAME_CONTROL_SOF_TMI_WIDTH)

        return frame_control

    def create_sack_frame_control(self, sackd):
        frame_control = bytearray(ieee1901.FRAME_CONTROL_NBITS/8);

        # Set delimiter type to SACK
        self.set_numeric_field(frame_control, 2, ieee1901.FRAME_CONTROL_DT_IH_OFFSET, ieee1901.FRAME_CONTROL_DT_IH_WIDTH);

        # SACK version number
        self.set_numeric_field(frame_control, 0, ieee1901.FRAME_CONTROL_SACK_SVN_OFFSET, ieee1901.FRAME_CONTROL_SACK_SVN_WIDTH);

        # Assign the SACK data bits
        sackd_padded = bytearray(ieee1901.FRAME_CONTROL_SACK_SACKD_WIDTH/8)
        sackd_padded[0:len(sackd)] = sackd;
        self.set_bytes_field(frame_control, sackd_padded, ieee1901.FRAME_CONTROL_SACK_SACKD_OFFSET/8, ieee1901.FRAME_CONTROL_SACK_SACKD_WIDTH/8)

        return frame_control;

    def create_sound_frame_control(self, pb_size):
        frame_control = bytearray(ieee1901.FRAME_CONTROL_NBITS/8);

        # Set delimiter type to Sound
        self.set_numeric_field(frame_control, 4, ieee1901.FRAME_CONTROL_DT_IH_OFFSET, ieee1901.FRAME_CONTROL_DT_IH_WIDTH);

        # Set the pbsz bit
        if pb_size == "PB520":
            self.set_numeric_field(frame_control, 0, ieee1901.FRAME_CONTROL_SOUND_PBSZ_OFFSET, ieee1901.FRAME_CONTROL_SOUND_PBSZ_WIDTH);
        else:
            self.set_numeric_field(frame_control, 1, ieee1901.FRAME_CONTROL_SOUND_PBSZ_OFFSET, ieee1901.FRAME_CONTROL_SOUND_PBSZ_WIDTH);

        return frame_control;

    def get_frame_type(self, frame_control):
        # Get delimiter type
        dt = self.get_numeric_field(frame_control, ieee1901.FRAME_CONTROL_DT_IH_OFFSET, ieee1901.FRAME_CONTROL_DT_IH_WIDTH);
        if (dt == 2):
            return "SACK"
        elif (dt == 1):
            return "SOF"
        elif (dt == 4):
            return "SOUND"
        else:
            sys.stderr.write (self.name + ": state = " + str(self.state) + ", unsupported frame type\n")
            return ""

    def set_bytes_field(self, frame, bytes, offset, width):
        frame[offset:(offset + width)] = bytes
        return offset + width

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
        return offset+width

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
