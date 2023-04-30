#!/usr/bin/env python3
#
# This file is in the public domain.
#
import argparse
import os
import serial
import struct
import sys
import time
import platform

from enum import IntEnum
from typing import List, Tuple


class Color(object):
	def __init__(self, r: int, g: int, b: int):
		self.r = r
		self.g = g
		self.b = b
	def to_bytes(self) -> bytes:
		return struct.pack("BBB", self.r, self.g, self.b)
	def __str__(self) -> str:
		return f"{self.r},{self.g},{self.b}"
	def __repr__(self) -> str:
		return f"Color('{self.r},{self.g},{self.b}')"
	def __mul__(self, rhs: int):
		return [Color(self.r, self.g, self.b) for i in range(rhs)]

def color_from_string(color_string):
	r, g, b = color_string.split(",")
	return Color(int(r), int(g), int(b))

def escape_bytes(b: bytes) -> bytes:
	result = []
	for byte in b:
		if byte == 0xd0 or byte == 0xe0:
			result.append(0xd0)
			result.append(byte - 1)
		else:
			result.append(byte)
	return bytes(result)

def unescape_bytes(b: bytes) -> bytes:
	result = []
	needs_escape = False
	for byte in b:
		if needs_escape:
			result.append(byte + 1)
			needs_escape = False
		else:
			if byte == 0xd0:
				needs_escape = True
			else:
				result.append(byte)
	return bytes(result)

class PRas3Exception(Exception):
	pass

# NFC
#
# Code to control a SEGA 837-15396/610-0955 NFC reader/writer.
#
#         1     2      3     4     5                        6
#      ,-----,------,-----,-----,-------------------------,------------,----------,
# 0xe0 | len | addr | seq | cmd | payload len (inclusive) | payload... | checksum |
#      '-----'------'-----'-----'-------------------------'------------'----------'
#  len includes checksum
#
# Expected rx sequence
#  0      1     2      3     4      5        6
# 0xe0 | len | addr | seq | cmd | status | payload len
#
# Commands:
#
# Info commands
# -------------
# 0x30: get FW version
# 0x32: get HW version
#
# General commands?
# -----------------
# 0x40: radio on
#   In order for MIFARE/FeliCa commands to work you must enable them here first.
#   arg0: 1, 2, or 3
#      1 = MIFARE
#      2 = FeliCa
#      3 = MIFARE + FeliCa
# 0x41: radio off
# 0x42: NFC poll
#	RESULT
#	- count
#	  - type: 0x10 (is MIFARE), 0x20 (is FeliCa)
#	  - UID
# 0x43: MIFARE select tag
#	arg0: 4-byte UID
# 0x44: also select tag?
#   arg0: can be longer than 4-byte key?

# MIFARE commands
# ---------------
# Need to turn the radio on for MIFARE to use these.
# 0x50: MIFARE set key B
#   arg0: 6-byte key
# 0x51: MIFARE auth Key B
#	This will also select the card.
#   arg0: 4-byte card UID
#	arg1: 1-byte block address
# 0x52: MIFARE read block
#	arg0: 4-byte block
#	arg1: 4-byte address
# 0x53: MIFARE write block
#	arg0: 4-byte card address
#	arg1: 1-byte block address
#	arg2: 16 bytes of data
# 0x54: MIFARE set key A
#   arg0: 6-byte key A
# 0x55: MIFARE authenticate key A
#	This will also select the card.
#   arg0: 4-byte card ID
#   arg1: 1-byte block address

# ????
# ----
# 0x60: Some kind of brief delay?

# FeliCa commands
# ---------------
# Need to turn the radio on for FeliCa to use these.
# 0x70: FeliCa thing
#      Sets flag 0x70 and do_FELICA
#	FeliCa command
#	arg0: 8-byte uid
#	arg1: FeliCa command bytes
#   code first sends [len[0xb0][UID][command bytes len][command bytes]]]
#   then sends       [len[0xa4][UID][0 byte]]
# 0x71: FeliCa thing 2
# 	IDm: manufacturer ID. 8 bytes as 16 ascii hex characters.
#      Sets flag 0x71 and do_FELICA
#	arg0: FeliCa command buffer, first byte is length
# 0x72: UNKNOWN
#	arg0: 1-byte something. 1 or 4
#	arg1: n-byte buffer
#
# Can use 0x71 to do some stuff like read Suica from Apple Pay transit card?? Only for octopus?
# https://github.com/metrodroid/metrodroid/wiki/IC-(Japan)
# https://github.com/metrodroid/metrodroid/wiki/FeliCa
# https://github.com/Seeed-Studio/PN532/blob/arduino/PN532/PN532.cpp
# https://github.com/lesterlo/Arduino-Octopus-Card-Reader/blob/master/Octopus_Reader/Octopus_Reader.ino
#
# An AmusementIC service code: 88 99
#
# RequestService:
# 0x02 (REQUEST_SERVICE)
# (8-byte IDm)
# service code request count (0x01)
# Little-Endian service code for Suica: 0x090f (History)
#
# Then...
# ReadWithoutEncryption:
# 0x06 (READ_WITHOUT_ENCRYPTION)
# (8-byte IDm)
# service code request count (0x01)
# Little-Endian service code for Suina History: 0x090f
# Block address count (0x01)
# block address...
#

# serviceCode=0x0117

# LED commands
# ------------
# 0x80: LED set channel
#      arg0: lower 3 bits bitfield
#			bit 0: set red
#			bit 1: set green
#			bit 2: set blue
#      arg1: color channel value
# 0x81: LED set color
#	arg0: red value
#	arg1: green value
#	arg2: blue value
# 0xf0: LED get info
# 0xf1: get HW version
# 0xf2: UNKNOWN
#
# Firmware internal state machine
# ----------------------
# state 0:
#  poll MIFARE -> state 1
#  poll FeliCa -> state 2
# state 1:
#  MIFARE auth B
#     if error? -> state 0
#	  if timeout? -> state 6
#	  if success? -> state 1
#  MIFARE auth A
#     if error? -> state 0
#	  if timeout? -> state 6
#	  if success? -> state 1
#  MIFARE read -> state 0
#  MIFARE write
#     if error -> state 1
#     if success -> state 0
#  FeliCa poll -> state 2
#  nothing -> state 0
# state 2:
#  cmd 0x70 -> state 0
#  cmd 0x71 -> state 3
# state 3:
#  cmd 0x70
#     ...
#  cmd 0x71
#     ...
# (unfinished)

class NFC:
	"""
	Be aware that you might need to put some sleeps between calling
	various commands. If you go too fast you'll get errors back.
	"""
	_seq: int

	class CardType(IntEnum):
		MIFARE = 1
		FeliCa = 2

	def __init__(self, port: str = None) -> None:
		if port is None:
			port = 'COM3' if platform.system() == 'Windows' else '/dev/ttyS2'
		self._ser = serial.Serial(port, 115200)
		self._seq = 0

	def _build_cmd(self, cmd: int, payload: bytes) -> bytes:
		addr = 0
		self._seq += 1
		buf = struct.pack("BBBB", addr, self._seq, cmd, len(payload)) + payload
		buf = struct.pack("B", len(buf) + 1) + buf # +1 for checksum
		checksum = struct.pack("B", sum(buf) % 256)
		buf = b'\xe0' + escape_bytes(buf + checksum)
		return buf

	def _get_response(self, debug: bool=False) -> bytes:
		"Reads a response and extracts the payload and status byte."
		# The structure of a response is:
		#
		# 0xe0 | len | addr | seq | cmd | status | payload len | n-byte payload | checksum
		#
		# With all fields being one byte except for the payload.

		sync = self._ser.read(1)
		assert(sync == b'\xe0')
		l = self._ser.read(1)
		l = struct.unpack("B", l)[0]
		buf = b''
		for i in range(l):
			b = self._ser.read(1)
			if b == b'\xd0':
				b = self._ser.read(1)
				b = bytes([struct.unpack("B",b)[0] + 1])
			buf += b

		buf = unescape_bytes(buf)

		addr, seq, cmd, status, payload_len = struct.unpack("BBBBB", buf[:5])
		payload = buf[5:-1]
		checksum = buf[-1]
		if debug:
			hex_buf = [f"{b:02x}" for b in buf]
			print(f"bytes: {hex_buf}")
			print(f"addr:{addr:02x} seq:{seq:02x} cmd:{cmd:02x} status:{status:02x} "
				f"payload len:{payload_len:02x} payload:{payload.hex()}")
		if status != 0:
			raise PRas3Exception(f"Got error response: {status}")
		return payload

	def reset(self) -> None:
		"""
		Need to call this after power on before you can do anything.
		Will give an error if given any other time.
		"""
		self._ser.write(self._build_cmd(0x62, b''))
		self._get_response()

	def get_firmware_version(self) -> bytes:
		# response: 0x94
		self._ser.write(self._build_cmd(0x30, b''))
		payload = self._get_response()
		return payload

	def get_hardware_version(self) -> bytes:
		# response: "837-15396" (version 3?)
		self._ser.write(self._build_cmd(0x32, b''))
		payload = self._get_response()
		return payload

	def radio_on(self, card_type: CardType) -> None:
		"Turns on the radio and starts scanning for tags of the given type."
		# 1 = MIFARE
		# 2 = FeliCa
		# You can technically scan for both at the same time by OR-ing them
		# together but we're not messing with that here.
		assert card_type == NFC.CardType.MIFARE or card_type == NFC.CardType.FeliCa
		self._ser.write(self._build_cmd(0x40, struct.pack("B", card_type)))
		self._get_response()

	def radio_off(self) -> None:
		"Turn the radio off when you're done. Not super important but let's be nice"
		self._ser.write(self._build_cmd(0x41, b''))
		self._get_response()

	def poll(self) -> List[Tuple[CardType, bytes]]:
		"""
		Get any cards in range of the reader. To scan for MIFARE
		cards you must be sure to turn the radio on in MIFARE mode.
		Likewise for FeliCa.
		If you call this too fast you'll get an error back.
		"""
		time.sleep(.15)
		self._ser.write(self._build_cmd(0x42, b''))
		buf = self._get_response()
		if len(buf) == 0:
			return []
		count = struct.unpack("B", buf[0:1])[0]
		buf = buf[1:]
		cards = []
		for i in range(count):
			card_type, size = struct.unpack("BB", buf[0:2])
			buf = buf[2:]
			if card_type == 0x10:
				# MIFARE cards have either 4, 7 or 10-byte UID (single, double, triple)
				# In 4-byte, leading byte of 0x8 means it's a random UID every time.
				# in 7-byte, UID0 is the manufacturer code.
				uid = buf[:size]
				cards.append((NFC.CardType.MIFARE, uid))
			elif card_type == 0x20:
				assert size == 16
				uid = buf[:size]
				cards.append((NFC.CardType.FeliCa, uid))
			else:
				# unknown card type
				assert False
			buf = buf[size:]
		return cards

	def MIFARE_select_tag(self, uid: bytes) -> None:
		"""
		Select the card to do operations on. You must do this before any other
		card-specific MIFARE operations.
		"""
		# I don't think it really matters that we use the right
		# command. We could use 0x43 for everything we'd just
		# need to truncate the UID before sending. This is
		# what the firmware does internally in 0x44.
		if len(uid) == 4:
			self._ser.write(self._build_cmd(0x43, uid))
		else:
			self._ser.write(self._build_cmd(0x44, uid))
		self._get_response()

	def MIFARE_set_key_A(self, key: bytes) -> None:
		assert len(key) == 6
		self._ser.write(self._build_cmd(0x54, key))
		self._get_response()

	def MIFARE_authenticate_key_A(self, uid: bytes, block: int) -> None:
		"""
		Select the card and authenticate against the block of interest with key A.
		You need to set key A before calling this.
		"""
		# MIFARE card consists of 16 sectors with 4 blocks each (total 16*4 = 64 blocks).
		# Each block has 16 bytes.
		# Before you can do any operations on a block
		# you must authenticate with that block's sector
		# Block 3 of every sector is the trailer.
		# this contains:
		# - secret keys A and B (optional) which return 0 when read.
		# - access conditions for blocks of that sector stored in bytes 6-9.
		#   Also access bits specify if data or value is stored.
		# If key B is not needed, the last 6 bytes of the trailer can be used as data bytes.
		# layout: key A | access bits | key B
		# sector 0 block 0 contains manufacturer data.
		self._ser.write(self._build_cmd(0x55, uid[:4] + struct.pack("B", block)))
		self._get_response()

	def MIFARE_set_key_B(self, key: bytes) -> None:
		assert len(key) == 6
		self._ser.write(self._build_cmd(0x50, key))
		self._get_response()

	def MIFARE_authenticate_key_B(self, uid: bytes, block: int) -> None:
		"""
		Select the card and authenticate against the block of interest with key B.
		You need to set key B before calling this.
		"""
		self._ser.write(self._build_cmd(0x51, uid[:4] + struct.pack("B", block)))
		self._get_response()

	def MIFARE_read_block(self, uid: bytes, block: int) -> bytes:
		"Reads a 16 byte block from the card given at the block address given"
		self._ser.write(self._build_cmd(0x52, uid[:4] + struct.pack("B", block)))
		payload = self._get_response()
		return payload

	def MIFARE_write_block(self, uid: bytes, block: int, block_data: bytes) -> None:
		assert len(block_data) == 16
		self._ser.write(self._build_cmd(0x53, uid[:4] + struct.pack("B", block) + block_data))
		self._get_response()

	def LED_set_channels(self, intensity, r: bool=False, g: bool=False, b: bool=False) -> None:
		"""
		Sets the indicated channels to the given intensity [0-255].
		Other channels are left unchanged.
		"""
		bits = (1 if r else 0) | (2 if g else 0) | (4 if b else 0)
		self._ser.write(self._build_cmd(0x80, struct.pack("BB", bits, intensity)))
		# no reply

	def LED_set_color(self, r: int, g: int, b: int) -> None:
		"Sets the color of the LEDs. All values are [0-255]"
		self._ser.write(self._build_cmd(0x81, struct.pack("BBB", r, g, b)))
		# no reply

	def LED_get_info(self) -> bytes:
		self._ser.write(self._build_cmd(0xf0, b''))
		payload = self._get_response()
		return payload

# LEDs
#
# Command format:
# 198 bytes of pixels = 66 pixels = 3 * 22 pixels
# ,-------------,-------------,-------------,--------------,----~              ~----,----------,
# | sync (0xe0) | dst node id | src node id | payload size | payload (199 byte max) | checksum |
# '-------------'-------------'-------------'--------------'----~              ~----'----------'
# payload structure
# ,-----,-------------------,
# | cmd | (up to 198 bytes) |
# '-----'-------------------'
#
# If send with a src node_id of 0 then you won't get a reply back.
#
# Reply format is the same except for the payload structure:
# payload structure
# ,--------,-----,---~         ~----------,
# | status | cmd | optional response data |
# '--------'-----'---~         ~----------'
#
# 22 pixels in the whole machine
# pixel buffer has enough space for three "pages" of pixels
#
# pixel layout:
# 16                       21
# 17                       20
# 18                       19
#
# 0                        15
# 1                        14
# 2                        13
#   3 4 5 6 7 8 9 10 11 12

class LEDs:
	#                  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21]
	LED_MAPPING    = [16, 17, 18,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 19, 20, 21]
	NORMAL_MAPPING = [ 3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18,  0,  1,  2, 19, 20, 21]
	def __init__(self, port=None):
		if port is None:
			port = "COM2" if platform.system() == "Windows" else "/dev/ttyS1"
		self._ser = serial.Serial(port, 115200)

	def _build_cmd(self, cmd: int, payload: bytes):
		# A destination of 0 acts like a wildcard. It won't matter what the real node id is
		# for the LEDs.
		dst_node_id = 0
		# Setting a source id of 0 silences replies.
		# If you want to get replies you need to set this to a non-zero value.
		src_node_id = 0
		buf = struct.pack("BBBB", dst_node_id, src_node_id, len(payload) + 1, cmd) + payload
		checksum = struct.pack("B", sum(buf) % 256)
		buf = b'\xe0' + escape_bytes(buf + checksum)
		return buf

	def _get_response(self, debug: bool=False) -> bytes:
		"Reads a response and extracts the payload and status byte."
		# The structure of a response is:
		#
		# 0xe0 | dst node | src node | payload len | n-byte payload | checksum
		#
		# With all fields being one byte except for the payload.

		sync = self._ser.read(1)
		assert(sync == b'\xe0')
		l = self._ser.read(1)
		l = struct.unpack("B", l)[0]
		buf = b''
		for i in range(3):
			b = self._ser.read(1)
			if b == b'\xd0':
				b = self._ser.read(1)
				b = bytes([struct.unpack("B",b)[0] + 1])
			buf += b

		dst, src, payload_length = struct.unpack("BBB", buf)

		payload = self._ser.read(payload_length)
		checksum = struct.unpack("B" , self._ser.read(1))
		payload = unescape_bytes(payload)

		status, cmd = struct.payload("BB", payload[:2])
		payload_body = payload[2:]
		if status != 1:
			raise PRas3Exception(f"Got error response: {status}")
		if debug:
			print(f"src:{src:02x} dst:{dst:02x} cmd:{cmd:02x} "
				f"status:{status:02x} payload len:{payload_length:02x} payload:{payload_body.hex()}")
		return payload_body

	def build_pixels(self, left_color: Color, center_color: Color, right_color: Color) -> bytes:
		"""
		Constructs a single screen's worth of pixels (22 pixels)
		"""
		pixels = left_color.to_bytes() * 6 + center_color.to_bytes() * 10 + right_color.to_bytes() * 6
		mapped = b''
		for i in range(22):
			pixel_i = self.NORMAL_MAPPING[i] * 3
			mapped += pixels[pixel_i:pixel_i + 3]
		return mapped

	# Unknown commands
	# 0x11 = <unknown>
	# 0x19 = <unknown>
	# 0xf3 = <unknown>

	def reset(self):
		"""
		Resets all the settings.
		"""
		# arg0: must be 217
		self._ser.write(self._build_cmd(0x10, struct.pack("B", 217)))

	def set_silent(self, is_silent):
		"""
		Put the device in to "silent" mode. In this mode it won't
		send responses to commands. You also get this behavior when
		sending the device commands with dest ID = 0.
		"""
		# arg0: <optional> enable or disable silent mode
		# not passing an argument will just get the current mode sent back.
		self._ser.write(self._build_cmd(0x14, struct.pack("B", is_silent)))

	def set_node_id(self, node_id):
		"""
		Sets the ID of the device.
		This is mostly useless. You can also set the ID with the DIP switches.
		node_id: (must be < 0x7f) Only 3 lsb used.
		"""
		assert node_id < 8 and node_id >= 0
		self._ser.write(self._build_cmd(0x18, struct.pack("B", node_id)))

	def draw_pixels(self):
		"""
		Just draws the pixels already in the buffer.
		see: set_pixels
		"""
		self._ser.write(self._build_cmd(0x80, b''))

	def set_pixels(self, pixel_buffer):
		"""
		Set pixels, but don't draw them.
		see: draw_pixels
		"""
		assert len(pixel_buffer) == 66*3
		self._ser.write(self._build_cmd(0x81, pixel_buffer))

	def set_and_draw_pixels(self, pixel_buffer):
		"""
		Immediately change to the pixel values sent.
		"""
		assert len(pixel_buffer) == 66*3
		self._ser.write(self._build_cmd(0x82, pixel_buffer))

	def fade_to_pixels(self, pixel_buffer):
		"""
		Fade to the pixels in the buffer.
		"""
		assert len(pixel_buffer) == 66*3
		self._ser.write(self._build_cmd(0x83, pixel_buffer))

	def set_blend_timing(self, frame_count, frame_delay):
		"""
		Sets how "smooth" a fade will be and how long it will take.
		frame_count: Defaults to 0x20 (can't be 0). The number of "frames"
		             it takes to blend from one color to the next.
		frame_delay: Defaults to 0x8 (can't be 0).
		             The amount of "frames" to delay each step of the blend.
		             This basically determines how long a "frame" is.
		"""
		# Send no arguments to get back the current values.
		assert frame_count > 0
		assert frame_delay > 0
		self._ser.write(self._build_cmd(0x84, struct.pack("BB", frame_count, frame_delay)))

	def do_offset_blend(self, offset):
		# Blends all pixels with pixel + offset from itself.
		# Restricted by window size set with "set window size" command.
		assert offset < 0x42
		self._ser.write(self._build_cmd(0x85, struct.pack("B", offset)))

	def set_blend_window_size(self, size):
		# Any size not 20 or 26 will generate an error AND success response
		# but will not set the window size.
		assert size == 20 or size == 26
		self._ser.write(self._build_cmd(0x86, struct.pack("B", size)))

	def set_blend_params(self, frame_count, offset):
		self._ser.write(self._build_cmd(0x87, struct.pack("BB", frame_count, offset)))

	def get_hw_name(self):
		# won't reply if dst_node_id is "alternate" (0) id
		self._ser.write(self._build_cmd(0xf0, b''))

	def get_board_state(self):
		self._ser.write(self._build_cmd(0xf1, b''))

	def get_code_checksum(self):
		self._ser.write(self._build_cmd(0xf2, b''))

	def enter_bootloader(self):
		self._ser.write(self._build_cmd(0xfd, b''))


class VFD:
	"""
	A class for controlling the Futaba GP1232A02A vacuum fluorescent display (VFD)
	that comes installed in the Sega P-RAS 3 arcade cabinet.
	The PCB names it GP1232A02 and Sega gives it part number 200-6275.

	At the time this script was written get_version() returned b'\x0201.20\x03'
	"""
	#
	# screen is 160 x 32
	# data is column-major with four bytes per column
	# Internal buffer is 512 wide
	#
	# Secondary buffer is 16 high x 752 wide.
	# large enough for 148 8x16 characters or 74 16x16 characters
	# Text draw command draws into this buffer.
	#

	class Encoding(IntEnum):
		GB2312    = 0
		BIG5      = 1
		SHIFT_JIS = 2
		KSC5601   = 3

	def __init__(self, port):
		# Requires hardware control flow.
		# RTS: Request To Send
		# CTS: Clear To Send
		# baud: 115200 18n
		speed = 115200
		# stopbits = serial.STOPBITS_ONE
		# parity = serial.PARITY_NONE
		# bytesize = serial.EIGHTBITS
		if port is None:
			port = 'COM1' if platform.system() == 'Windows' else '/dev/ttyS0'
		self._ser = serial.Serial(port, speed, rtscts=True)
		self._encoding = VFD.Encoding.SHIFT_JIS

	def __encode(self, s: str):
		if self._encoding == VFD.Encoding.GB2312:
			return s.encode("GB2312")
		elif self._encoding == VFD.Encoding.BIG5:
			return s.encode("BIG5")
		elif self._encoding == VFD.Encoding.SHIFT_JIS:
			return s.encode("shift-jis")
		elif self._encoding == VFD.Encoding.KSC5601:
			return s.encode("KSC5601")
		else:
			raise Exception(f"Unknown encoding {self._encoding}")

	def text(self, s: bytes):
		"""
		Draw text into the background plane at the current
		cursor position. Drawing text automatically advances the cursor.
		"""
		# 0x20 - 0xFF: text
		self._ser.write(self.__encode(s))

	#
	# Commands starting with 0x1b (ESC):
	#

	def reset(self):
		"""
		Reset the VFD state.
		"""
		# - 0xc: reset state
		#   - args = 0
		b = b'\x1b\x0b'
		self._ser.write(b)
		self._encoding = VFD.Encoding.SHIFT_JIS

	def clear_screen(self):
		"""
		Clear the main screen.
		"""
		b = b'\x1b\x0c'
		self._ser.write(b)

	def set_brightness(self, level: int):
		"""
		Brightness: 0-4
		0 is off
		"""
		# - 0x20: ??
		#   - args = 1 : 0-4
		b = b'\x1b\x20' + struct.pack(">B", level)
		self._ser.write(b)

	def turn_on(self, on: bool):
		"""
		Turn screen on.
		"""
		# - 0x21: turn screen on
		#   - args = 1 : 0 or 1
		b = b'\x1b\x21' + struct.pack(">B", on)
		self._ser.write(b)

	def set_window_h_scroll(self, x: int):
		"""
		Set horizontal scroll of the background plane.
		"""
		# - 0x22: set window h position
		#   - args = 1
		#	  x pos?
		assert x < 512
		b = b'\x1b\x22' + struct.pack(">H", x)
		self._ser.write(b)

	def draw_bitmap(self, x_start, y_start, w, h, bitmap):
		"""
		Draw a bitmap into the background plane.
		The bitmap is "column-major" 1 bit-per-pixel.
		The "h" parameter is assumed to be in bytes and not pixels
		so you likely need to divide it by 8 before passing it in.
		"""
		# - 0x2e: fill pixels
		#   - args = 6
		#     x start u16-be
		#     y start u8
		#     x end   u16-be
		#     y end   u8 (this one is checked "<" hence the "- 1"
		#     (then bitmap data)
		assert x_start < 512
		assert x_start + w <= 512
		assert y_start < 4
		assert y_start + h <= 4
		assert len(bitmap) == h * w

		b = b'\x1b\x2e' + struct.pack(">HBHB", x_start, y_start, w, y_start + h - 1)
		b += bitmap
		self._ser.write(b)

	def set_cursor_pos(self, x: int, y: int):
		"""
		Put the text cursor at the given position.
		The next text command will draw character starting at this position.
		"""
		# - 0x30: set window pos ??
		#   - args = 3
		#     x pos: u16-be
		#     y pos: u8
		assert x < 512 and y < 3
		b = b'\x1b\x30' + struct.pack(">HB", x, y)
		self._ser.write(b)

	def set_text_encoding(self, encoding: Encoding):
		"""
		text encoding: 0-3
		0: GB2312
		1: BIG5
		2: SHIFT-JIS
		3: KSC5601
		"""
		# - 0x32: set text encoding
		#   - args = 1
		#     arg0: byte 0-3 : selects text decoding function
		assert encoding in range(4)
		self._encoding = encoding
		b = b'\x1b\x32' + struct.pack(">B", encoding.value)
		self._ser.write(b)

	def set_text_window(self, x, y, w):
		"""
		Place a scrolling text window on display.
		Doesn't show until you start scrolling.
		"""
		# - 0x40
		#   - args = 6
		#     x: u16-be
		#     y: u8
		#     x end: u16-be
		#     <ignored>: u8
		b = b'\x1b\x40' + struct.pack(">HBHB", x, y, w, 0)
		self._ser.write(b)

	def set_text_scroll_speed(self, speed: int):
		"""
		Text scroll speed: 0 - 1.
		0 is fastest.
		"""
		# - 0x41: text y pos
		#   - args = 1
		#     y: u8
		assert speed in [0,1]
		b = b'\x1b\x41' + struct.pack(">B", speed)
		self._ser.write(b)

	def write_scroll_text(self, chars: str):
		"""
		Write text into the scroll buffer.
		"""
		# - 0x50: draw chars into buffer
		#   - args = 1: 0x1-0x94 (148). Number of characters
		#	  (then data... characters to draw)
		assert len(chars) > 0 and len(chars) < 0x95
		# TODO: encode from string based on encoding
		char_bytes = self.__encode(chars)
		b = b'\x1b\x50' + struct.pack("B", len(char_bytes)) + char_bytes
		self._ser.write(b)

	def set_text_scroll(self, enable: bool):
		"""
		Enable/Disable text scroll.
		"""
		# - 0x51: enable text scroll
		# - 0x52: disable text scroll
		if enable:
			b = b'\x1b\x51'
		else:
			b = b'\x1b\x52'
		self._ser.write(b)

	def get_version(self):
		"""
		Get the firmware version.
		"""
		# - 0x5b: get version string
		#   - args = 1
		#     arg0: byte. Must be 99
		b = b'\x1b\x5b' + struct.pack("B", 99)
		self._ser.write(b)
		return self._ser.read(7)

	def flip_xy(self, flip: bool):
		"""
		Flip screen in X and Y (180 degree rotate).
		"""
		# - 0x5d: XY flip
		#   - args = 1
		#     arg0: byte 0x72 (114) or 0x6e (110)
		v = 114 if flip else 110
		b = b'\x1b\x5d' + struct.pack("B", v)
		self._ser.write(b)

	#
	# commands starting with 0x1a (NB):
	#

	def load_16x16_char(self, index: int, text: str):
		"""
		Store a 16x16 character image into one of 16 slots.
		I have no idea how you indicate what character it
		substitutes for.
		"""
		# - 0xa3 (state 0x14)
		#   - args = 1
		#     arg0: byte 0-15
		#     (32 bytes)
		assert index in range(16)
		assert len(bits) == 32
		b = b'\x1a\xa3' + struct.pack("B", index) + bits
		self._ser.write(b)

	def load_8x16_char(self, index: int, char: int, bits: bytes):
		"""
		Load an 8x16 character font bitmap. Will be substituted
		for the given 8-bit character when drawing strings.
		"""
		# - 0xa4 (state 0x15)
		#   - args = 2
		#     arg0: byte 0-15
		#     arg1: char
		#     (16 bytes)
		assert index in range(16)
		assert char <= 0xff
		assert len(bits) == 16
		b = b'\x1a\xa4' + struct.pack("BB", index, char) + bits
		self._ser.write(b)

	@classmethod
	def rotate_bitmap(self, b: bytes, width: int, height: int):
		"""
		Takes a 1-bit image with bytes oriented horizontally and rotates it to
		a vertical, column-major orientation required by the P-Ras VFD.

		input:
				byte 0            byte 1
		row 0: [7 6 5 4 3 2 1 0] [7 6 5 4 3 2 1 0] ...
		row 1: [7 6 5 4 3 2 1 0] [7 6 5 4 3 2 1 0] ...
			   ...

		output:
				col 0 (x=0) col 1 (x=1) ...
		byte 0: 7           7
		  y = 1 6           6
		  y = 2 5           5
		  ...   4           4
				3           3
				2           2
				1           1
				0           0
		byte 1: 7           7
				6           6
				5           5
				4           4
				3           3
				2           2
				1           1
				0           0
		"""
		rotated = b''
		w = width // 8
		for x in range(width):
			byte_x = x // 8
			bit_x = x % 8
			for y in range(height // 8):
				# grab a block of 8 bytes
				block = [
					b[(y*8 + 0) * w + byte_x],
					b[(y*8 + 1) * w + byte_x],
					b[(y*8 + 2) * w + byte_x],
					b[(y*8 + 3) * w + byte_x],
					b[(y*8 + 4) * w + byte_x],
					b[(y*8 + 5) * w + byte_x],
					b[(y*8 + 6) * w + byte_x],
					b[(y*8 + 7) * w + byte_x],
				]
				shift = 7 - bit_x
				rotated += bytes([
					((block[0] >> shift) & 1) << 7 |
					((block[1] >> shift) & 1) << 6 |
					((block[2] >> shift) & 1) << 5 |
					((block[3] >> shift) & 1) << 4 |
					((block[4] >> shift) & 1) << 3 |
					((block[5] >> shift) & 1) << 2 |
					((block[6] >> shift) & 1) << 1 |
					((block[7] >> shift) & 1) << 0
				])
		return rotated

	@classmethod
	def draw_bitmap_normal(self, b: bytes, width: int, height: int):
		"""
		Draws a row-major bitmap as ascii characters to stdout.
		"""
		for y in range(height):
			byte_y = y // 8
			bit_y = y % 8
			for x in range(width):
				byte_x = x // 8
				bit_x = x % 8
				p = b[y * (width//8) + byte_x]
				sys.stdout.write('0' if (p & (1 << (7 - bit_x))) else ' ')
			sys.stdout.write('\n')

	@classmethod
	def draw_bitmap_rotated(self, b: bytes, width: int, height: int):
		"""
		Draws a column-major bitmap as ascii characters to stdout.
		"""
		for y in range(height):
			byte_y = y // 8
			bit_y = y % 8
			for x in range(width):
				byte_x = x // 8
				bit_x = x % 8

				p = b[x * height//8 + byte_y]
				sys.stdout.write('0' if (p & (1 << (7 - bit_y))) else ' ')
			sys.stdout.write('\n')

	@classmethod
	def convert_ascii_art(self, lines: List[str]):
		"""
		Takes an array of text lines and transforms it into a row-major bitmap.
		It expects empty trailing lines to be removed already.

		returns: (width, height, bytes)
		Run the "rotate" function to get a column-major bitmap suitable for
		the P-Ras.
		"""
		lines = [l.strip("\n") for l in lines]
		width = max([len(l) for l in lines])
		height = len(lines)
		assert height % 8 == 0

		result = b''
		for l in lines:
			x = 0
			bit = 7
			byte = 0
			for c in l:
				if c != ' ':
					byte = byte | (1 << bit)
				if bit == 0:
					result = result + bytes([byte])
					bit = 7
					byte = 0
				else:
					bit = bit - 1
			if bit != 7:
				result = result + bytes([byte])
				byte = 0
		return (width, height, result)


def do_led(args):
	leds = LEDs(args.port)
	left_color = args.left or args.color
	right_color = args.right or args.color
	center_color = args.center or args.color

	b = leds.build_pixels(left_color, center_color, right_color)
	leds.fade_to_pixels(b * 3)

def do_nfc(args):
	nfc = NFC(args.port)
	try:
		nfc.reset()
	except PRas3Exception as e:
		# If we've already done this once after power on then we'll get this error.
		# But we can ignore it.
		pass
	if args.color is not None:
		nfc.LED_get_info()
		nfc.LED_set_color(args.color.r, args.color.g, args.color.b)

	nfc.radio_on(NFC.CardType.MIFARE)
	time.sleep(0.5)

	start = time.time()
	found_uids = set()
	while args.timeout is None or (time.time() - start) < args.timeout:
		cards = nfc.poll()
		for card_type, card_uid in cards:
			if card_uid in found_uids:
				continue
			if card_uid == args.wait_for_specific:
				return 0
			print(card_uid.hex())
			found_uids.add(card_uid)
			if args.wait_for_specific is None and len(found_uids) >= args.wait_for_any:
				return 0
		time.sleep(0.25)
	return 1

def do_vfd(args):
	vfd = VFD(args.port)

	if args.reset:
		vfd.reset()
	vfd.turn_on(not args.off)
	if args.text is not None:
		if args.text:
			vfd.set_text_window(20, 1, 120)
			vfd.write_scroll_text(args.text)
			vfd.set_text_scroll(True)
		else:
			vfd.set_text_scroll(False)
	if args.image is not None:
		image_path = args.image
		with open(args.image, "r") as f:
			lines = f.readlines()
		if lines[-1] == "":
			lines.pop()
		w, h, image = vfd.convert_ascii_art(lines)
		image = vfd.rotate_bitmap(image, w, h)
		vfd.draw_bitmap(0, 0, w, h//8, image)
	if args.brightness is not None:
		vfd.set_brightness(args.brightness)


def bytes_from_string(s: str):
	if s.startswith("0x") or s.startswith("0X"):
		s = s[2:]
	if len(s) % 2:
		raise Exception("UID must have even number of hex digits")
	uid = b''
	for i in range(len(s) // 2):
		byte_string = s[i*2:i*2+2]
		value = int(byte_string, 16)
		uid += bytes([value])
	return uid

if __name__ == "__main__":
	is_windows = platform.system() == "Windows"
	parser = argparse.ArgumentParser()
	subparsers = parser.add_subparsers(dest='device')
	subparsers.required = True
	led_parser = subparsers.add_parser('led')
	led_parser.add_argument("--port", help="serial port", default='COM2' if is_windows else '/dev/ttyS1')
	led_parser.add_argument("--color", type=color_from_string, help="color to set LEDs to", default=Color(255,0,0))
	led_parser.add_argument("--left", type=color_from_string, help="color to set left LEDs to")
	led_parser.add_argument("--right", type=color_from_string, help="color to set right LEDs to")
	led_parser.add_argument("--center", type=color_from_string, help="color to set center LEDs to")
	led_parser.set_defaults(func=do_led)

	nfc_parser = subparsers.add_parser('nfc')
	nfc_parser.add_argument("--port", help="serial port", default='COM3' if is_windows else '/dev/ttyS2')
	nfc_parser.add_argument("--color", type=color_from_string, help="LED color", metavar='<0-255>,<0-255>,<0-255>')
	nfc_parser.add_argument("--wait_for_specific", type=bytes_from_string, help="wait for a specific UID", metavar='<ffffffff>')
	nfc_parser.add_argument("--wait_for_any", default=1, type=int, help="read N unique UIDs", metavar='N')
	nfc_parser.add_argument("--timeout", type=int, help="Amount of time to wait. Default is forever")
	nfc_parser.set_defaults(func=do_nfc)

	vfd_parser = subparsers.add_parser('vfd')
	vfd_parser.add_argument("--port", help="serial port", default='COM1' if is_windows else '/dev/ttyS0')
	vfd_parser.add_argument("--text", help="Text to scroll. English or Japanese only. Use empty string to turn off.")
	vfd_parser.add_argument("--image", help="Text file with image to use as background.")
	vfd_parser.add_argument("--brightness", type=int, help="Brightness level (0-4).")
	vfd_parser.add_argument("--off", action='store_true', default=False,
		help="Turn the display off. All other commands turn the screen on implicitly.")
	vfd_parser.add_argument("--reset",
		help="Clear and reset the display state. This will be done before any other operations",
		action='store_true')
	vfd_parser.set_defaults(func=do_vfd)

	args = parser.parse_args(sys.argv[1:])
	sys.exit(args.func(args))
