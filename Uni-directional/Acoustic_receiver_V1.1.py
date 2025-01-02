#!/usr/bin/env python
#libs
import numpy as np
import scipy.signal
import pyaudio
import time
import os

#receiver class
class receiver:
	#initializing variables
	def __init__(self):
		self.CRC_poly = np.uint8(220)	#CRC key
		self.preamble = np.uint8(170)	#packet initial/preamble
		self.pow_switch = 0				#power button 0-> ON & 1-> OFF
		self.timer = 600				#live for 2 minute 
		self.fs = 48000					#Audio sampling frequency 
		self.fc = 18600					#mixing frequency... f0 = 200 & f1 = 600
		self.buffer_len = self.fs/2		#half a second
		self.lo = 2 * np.cos(2*np.pi*(self.fc/np.float32(self.fs))*np.arange(0,80,1))
		self.phase_pointer = 0			#self.lo array pointer

		self.filter_order = 100			#filter order
		self.filter_coff = scipy.signal.firwin(self.filter_order,
			cutoff = 800/24000.0,
			window = "hamming")			# 800 Hz low pass

		#variables for shapping signal to bits
		self.decimate_fac = 4
		self.last_samples = np.zeros(99)
		self.bit_period = 240
		self.previous = 0
		self.crossing_count = 0
		self.index_period = 0 
		self.bit_collect = np.uint8([])
		self.gap = True
		self.tune = False
		self.delta_crossing = 0
		self.warning = 0

		#record data
		self.eof_pkt_no = 0
		self.data_rec = {}
		self.writing = open('received message','a+')
		self.AGC_out = 32767
		self.command_finder = ''

	#CRC check 
	def CRC_R(self,word):
		dividend = np.uint8(word[0])
		bits_values = word[1:]
		temp_bits = np.unpackbits(bits_values)
		for i in range (0,len(temp_bits)):
			if(dividend>127): #10000000
				dividend = dividend^self.CRC_poly
			dividend = dividend<<1
			if(temp_bits[i]==1):
				dividend +=np.uint8(1)
		return dividend

	#extract packet data
	def pkt_data(self,data):
		temp_string = ''
		pkt_no = 0
		if (0 == self.CRC_R(data)):
			pkt_no = data[0]
			for i in range(1,6):
				temp_string = temp_string + chr(data[i])
		return temp_string, pkt_no

	#Recover packet and extract the message 
	def recover_data(self,raw_data):
		i = 0
		while (i<len(raw_data)-7):
			if (self.preamble == np.packbits(raw_data[i:i+8])):
				if(i+63 < len(raw_data)):
					frame_data, p_no = self.pkt_data(np.packbits(raw_data[i+8:i+64]))
					if (len(frame_data) != 0):
						self.writing.write(frame_data + ',' + str(p_no) + '\n')
						print(frame_data)
						if (frame_data == "#####"):
							if (self.eof_pkt_no != 0 and self.eof_pkt_no != p_no):
								self.data_rec.clear()
							self.eof_pkt_no = p_no
						else:
							if (self.data_rec.has_key(p_no)):
								if (self.data_rec[p_no] != frame_data):
									self.data_rec.clear()
							else:
								self.data_rec[p_no] = frame_data
						self.print_message()
						i +=64
					else:
						i += 1
				else:
					return raw_data[i:]
			else:
				i +=1
		return raw_data[i:]

	#print message and reintialize variable
	def print_message(self):
		message_0 = ''
		if (self.eof_pkt_no != 0):
			i = 0
			loop = True
			while (loop and i < self.eof_pkt_no):
				if (self.data_rec.has_key(i)):
					message_0 = message_0 + self.data_rec[i]
					loop = True
				else:
					loop = False
				i += 1
			if (i == self.eof_pkt_no):
				self.eof_pkt_no = 0
				self.data_rec.clear()
				self.writing.write('\n#Complete message#\n' + message_0)
				print('\n#Complete message#\n' + message_0)

	def AGC_control(self,max_val):
		if (max_val > 0.8):
			self.AGC_out -= 1000
		elif (max_val < 0.8 and self.AGC_out < 65535):
			self.AGC_out += 1000
		else:
			pass
		os.system("pacmd set-source-volume "+ self.command_finder[-1][0] +" "+ str(self.AGC_out))

	#run after 0.5 second, take microphone data
	def callback(self ,in_data, frame_count, time_info, status):
		instant_data = np.fromstring(in_data,np.float32)
		self.AGC_control(np.amax(abs(instant_data)))
		for i in range(0,len(instant_data)):
			instant_data[i] = instant_data[i]* self.lo[self.phase_pointer%80]
			self.phase_pointer = (self.phase_pointer + 1) %80
		instant_data = np.append(self.last_samples,instant_data,axis=0)
		self.last_samples = instant_data[len(instant_data)-99:]
		instant_data = np.convolve(instant_data,self.filter_coff,'valid')

		for i in range(0,len(instant_data)):
			temp = instant_data[i]
			self.index_period +=1
			if (temp >= 0 )^(self.previous >= 0):
				self.crossing_count += 1
				self.previous = temp

			if (self.index_period == self.bit_period):
				extract_bit = np.uint8(0)
				if(self.crossing_count > 4):
					extract_bit = np.uint8(1)
				self.bit_collect = np.append(self.bit_collect,[extract_bit],axis= 0)
				self.index_period = 0
				self.crossing_count = 0
		self.bit_collect = self.recover_data(self.bit_collect)
		data = np.zeros(frame_count,dtype=np.float32)
		return data, self.pow_switch

	#running body of the application/script
	def prog(self):
		digital_in = pyaudio.PyAudio()
		self.command_finder =  os.popen("pactl list short sources").readlines()
		os.system("pacmd set-source-port "+ self.command_finder[-1][0] +" analog-input-headphone-mic")
		os.system("pacmd set-source-volume "+ self.command_finder[-1][0] +" "+ str(self.AGC_out))
		in_audio = digital_in.open(format=pyaudio.paFloat32,
			channels=1,
			rate=self.fs,
			frames_per_buffer=self.buffer_len,
			input=True,
			output=False,
			stream_callback = self.callback,
			)
		start_time = time.time()
		while in_audio.is_active():
			if (time.time() > start_time + self.timer):
				self.pow_switch = 1
				time.sleep(0.1)
				pass
		in_audio.stop_stream()
		in_audio.close()
		digital_in.terminate()
		self.writing.close()

if __name__ == '__main__':
	receiver_data = receiver()
	receiver_data.prog()
