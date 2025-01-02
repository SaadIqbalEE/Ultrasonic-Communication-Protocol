#!/usr/bin/env python
import scipy.signal
import numpy as np
import pyaudio
import time
import threading
import os

class transceiver:

	def __init__(self):
		self.start_time = 0
		self.pow_switch = 0		#system switch
		self.timer = 300		#5 min....change to 600
		self.work_mode = True	#True for transmitting
		self.new_point = False	#New comm. start
		self.initial_time = 0	#reception Start time
		self.bytesperpacket = 4	#characters per pkt
		self.CRC_poly = np.uint8(213)
		self.pre = np.uint8(170)		#preamble
		self.prem = np.uint8(171)
		self.dis = np.array(241,dtype=np.uint8)		#discovery
		self.dck = np.array(227,dtype=np.uint8)		#discovery acknowledge
		self.ack = np.array(143,dtype=np.uint8)		#data acknowledge
		self.ret = np.array(157,dtype=np.uint8)		#data retransmit
		self.tor = np.array(199,dtype=np.uint8)		#token release
		self.tck = np.array(185,dtype=np.uint8)		#token release acknowledge
		self.fs = 48000
		self.fc = 18600
		self.f0 = 18800
		self.f1 = 19200
		self.bit_samples = 240
		self.buf_len_R = self.fs/4
		self.buf_len_T = self.fs/6
		self.D1 = np.cos((2*np.pi*(self.f0/np.float32(self.fs))*np.arange(0,self.bit_samples,1)), dtype=np.float32)
		self.D2 = np.cos((2*np.pi*(self.f1/np.float32(self.fs))*np.arange(0,self.bit_samples,1)), dtype=np.float32)
		self.lo = np.cos((2*np.pi*(self.fc/np.float32(self.fs))*np.arange(0,80,1)), dtype=np.float32)
		self.phase_pointer = 0
		self.filter_order = 100
		self.filter_coff = np.float32(scipy.signal.firwin(self.filter_order, cutoff = 800/24000.0,	window = "hamming"))
		self.last_samples = np.zeros(99)
		self.decimate_fac = 4
		self.threshold = 0
		self.previous = 0
		self.crossing_count = 0
		self.index_period = 0 
		self.bit_collect = np.uint8([])
		
		self.take_input = True
		self.rawdata_snd = ['server']
		self.rawdata_snd_ptr = 0
		self.string_data = []
		self.string_data_ptr = 0
		self.w_alter_bit = 0
		self.r_alter_bit = 0
		self.data_rec = ''
		self.last_pkt = ''
		self.idle_time = 3
		self.instant_transmit = np.float32([])
		self.transmit_ptr = 0	#pointer for transmitting data
		self.padata = np.float32([])
		
		self.w_dat = False
		self.w_dis = False
		self.w_dck = False
		self.w_ack = False
		self.w_ret = False
		self.w_tor = False
		self.w_tck = False
		
		self.D_bit = False		#connection eastablished or not

		self.current_state = 0
		self.next_state = 0
		self.is_interrupt = False

		self.cntrl_m = 1		# 0-> No send
								# 1-> Data senc
								# 2-> Discovery/broadcast message
								# 3-> Ack_discovery
								# 4-> Acknowledge message
								# 5-> retransmission message
								# 6-> token released message
								# 7-> Ack_token

		self.no_listen = 45
		self.last_comm = time.time()
		self.dis_listen = [2,3,4]		#make modulo of 3
		self.dis_listen_p = 0
		self.cntrl_r_pkt_listen = 3

		self.dack = False		#data without alteration arrived
		self.dret = False		#data with alteration arrived
		self.disc = False		#discovery message arrived
		self.diac = False		#discovery acknowledge message arrived
		self.ackn = False		#acknowledge message arrived
		self.retr = False		#retransmit message arrived
		self.tokr = False		#token released message arrived
		self.tack = False		#token released acknowledged messaged arrived


		self.states = { 0 : self.s_ndis,
		1 : self.s_dtor,
		2 : self.s_pkts,
		3 : self.s_ta1k,
		4 : self.s_ta2k,
		5 : self.s_pktr,
		}
		self.Digital_control = pyaudio.PyAudio()

		self.recording = open('record message_S','a+')
		self.command_finder_0 = ''
		self.command_finder_1 = ''		
		self.AGC_out = 32767
		
	#CRC calculation
	def CRC(self,payload,state_bit):
		dividend = np.uint8(payload[0])
		if (state_bit == True):
			bits_values =np.append(payload[1:],[np.uint8(0)],axis=0)
		else:
			bits_values = payload[1:]
		temp_bits = np.unpackbits(bits_values)
		temp_bits[-1] = 0
		for i in range (0,len(temp_bits)):
			if(dividend>127):	#10000000
				dividend = dividend^self.CRC_poly
			dividend = dividend<<1
			if(temp_bits[i]==1):
				dividend +=np.uint8(1)
		return dividend

	#string data as string_data
	def rawtopack(self,string_data):
		f_pointer = 0
		f_length = len(string_data)
		modulo_size = (f_length + 1) % self.bytesperpacket
		num_null = 0 if modulo_size == 0 else self.bytesperpacket-(modulo_size)
		pack_size = ((f_length + 1 + num_null)/ self.bytesperpacket)
		raw_pack = np.zeros(pack_size * (self.bytesperpacket + 4),dtype=np.uint8)
		for j in range (0,pack_size):
			temp = (j*8) + 1
			raw_pack[temp] = self.pre
			temp += 1
			for i in range(0,self.bytesperpacket):
				if (f_pointer < f_length):
					raw_pack[temp+i] = ord(string_data[f_pointer])
				elif (j == pack_size - 1 and i == self.bytesperpacket - 1):
					raw_pack[temp+i] = ord('\n')
				else:
					raw_pack[temp+i] = ord(' ')
				f_pointer += 1
			if (self.w_alter_bit == 0):
				raw_pack[temp+4] = self.CRC(raw_pack[temp:temp+4],True) + 1
				self.w_alter_bit = 1
			else:
				raw_pack[temp+4] = self.CRC(raw_pack[temp:temp+4],True)
				self.w_alter_bit = 0
		return self.modulation_cpfsk(np.unpackbits(raw_pack))

	#cntrl_m as value
	def control_pkt(self,value): 
		C_pkt = np.zeros(4,dtype=np.uint8)
		C_pkt[1] = self.prem
		if (value == 2):
			C_pkt[2] = self.dis
		elif (value == 3):
			C_pkt[2] = self.dck
		elif (value == 4):
			C_pkt[2] = self.ack			
		elif (value == 5):
			C_pkt[2] = self.ret
		elif (value == 6):
			C_pkt[2] = self.tor
		elif (value == 7):
			C_pkt[2] = self.tck					
		else:
			C_pkt[2] = np.uint8(0)
		C_pkt[3] = self.CRC(np.uint8([C_pkt[2]]),True)
		return self.modulation_cpfsk(np.unpackbits(C_pkt))

	##mod m = 2 & mod_data is binary data
	def modulation_cpfsk(self,mod_data):
		transmit_data = np.zeros(len(mod_data)*self.bit_samples,dtype=np.float32)
		for i in range(0,len(mod_data)):
			if(mod_data[i]):
				transmit_data[i*self.bit_samples:(i+1)*self.bit_samples]= self.D2
			else:
				transmit_data[i*self.bit_samples:(i+1)*self.bit_samples]= self.D1
		return transmit_data

	#received packet read
	def pkt_data(self,data):
		temp_string = ''
		if (0 == self.CRC(data,False)):
			for i in range(0,4):
				temp_string = temp_string + chr(data[i])
		return temp_string

	def recover_data(self,raw_bits):
		i = 0
		interrupt_call = False
		while (i+7 < len(raw_bits)):
			if (self.pre == np.packbits(raw_bits[i:i+8]) and self.w_dat):
				if(i+47 < len(raw_bits)):
					frame_data = self.pkt_data(np.packbits(raw_bits[i+8:i+48]))
					if (len(frame_data) != 0 ):
						self.dack = True #pkt acknowledge
						interrupt_call = True
						if (frame_data == self.last_pkt and self.r_alter_bit == raw_bits[i+47]):
							pass
						else:
							self.last_pkt = frame_data
							self.r_alter_bit = raw_bits[i+47]
							if (frame_data[-1] == '\n'):
								self.recording.write('\n#Complete message#\n' + self.data_rec + frame_data)
								self.terminal_output('\n#Complete message#\n' + self.data_rec + frame_data)
								self.data_rec = ''
							else:
								self.data_rec = self.data_rec + frame_data
						return np.uint8([]) , interrupt_call
					else:
						i += 1
				else:
					return raw_bits[i:] , interrupt_call
			elif (self.prem == np.packbits(raw_bits[i:i+8])):
				if(i+23 < len(raw_bits)):
					cnt_data = np.packbits(raw_bits[i+8:i+16])
					if (0 == self.CRC(np.packbits(raw_bits[i+8:i+24]),False)):
						if (self.dis == cnt_data and self.w_dis):
							self.disc = True  #discover acknowledge
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						elif (self.dck==cnt_data and self.w_dck):
							self.diac =  True #discover acknowledge*
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						elif (self.ack==cnt_data and self.w_ack):
							self.ackn = True
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						elif (self.ret==cnt_data and self.w_ret):
							self.retr = True
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						elif (self.tor==cnt_data and self.w_tor):
							self.tokr = True
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						elif (self.tck==cnt_data and self.w_tck):
							self.tack = True
							interrupt_call = True
							return np.uint8([]) , interrupt_call
						else:
							pass
					else:
						pass
					i += 1
				else:
					return raw_bits[i:] , interrupt_call
			else:
				i += 1
		return raw_bits[i:] , interrupt_call

	def callback(self, in_data, frame_count, time_info, status):
		self.padata = np.zeros(frame_count,dtype=np.float32)
		interrupt = False
		switch = 0
		if (self.work_mode):
			self.padata, interrupt = self.proc_snd(self.new_point, frame_count)
			if (interrupt):
				switch = 1
		else:
			instant_data = np.fromstring(in_data,np.float32)
			self.AGC_control(np.amax(abs(instant_data)))
			interrupt = self.proc_rec(instant_data, self.new_point, frame_count)
			if ((time.time() > self.initial_time + self.idle_time) or interrupt):
				switch = 1
		self.new_point = False
		return self.padata, switch

	def proc_snd(self, n_point, chunck):
		ini = False
		if (n_point == True):
			self.transmit_ptr = 0
		if ((len(self.instant_transmit) - self.transmit_ptr) > chunck):
			snd_data = self.instant_transmit[self.transmit_ptr:self.transmit_ptr + chunck]
			self.transmit_ptr += chunck
			return snd_data , ini
		else:
			snd_data =np.concatenate([self.instant_transmit[self.transmit_ptr:],
				np.zeros(chunck-(len(self.instant_transmit)-self.transmit_ptr),dtype= np.float32)],
				axis=0)
			ini = True
			return snd_data , ini

	def proc_rec(self,dump_data, n_point, chunck):
		ini =False
		if (n_point == True):
			self.phase_pointer = 0
			self.last_samples = np.zeros(99)
			self.index_period = 0
			self.previous = 0
			self.crossing_count = 0
			self.bit_collect = np.uint8([])

		for i in range(0,chunck):
			dump_data[i] = dump_data[i]* self.lo[self.phase_pointer%80]
			self.phase_pointer = (self.phase_pointer + 1) %80
		dump_data = np.append(self.last_samples,dump_data,axis=0)
		self.last_samples = dump_data[len(dump_data)-99:]
		dump_data = np.convolve(dump_data,self.filter_coff,'valid')

		for i in range(0,len(dump_data)/self.decimate_fac):
			temp = dump_data[i*self.decimate_fac]
			self.index_period +=1
			if (temp >= self.threshold )^(self.previous >= self.threshold):
				self.crossing_count += 1
				self.previous = temp
			if (self.index_period == self.bit_samples/self.decimate_fac):
				extract_bit = np.uint8(0)
				if (self.crossing_count > 4):
					extract_bit = np.uint8(1)
				self.bit_collect = np.append(self.bit_collect,[extract_bit],axis= 0)
				self.index_period = 0
				self.crossing_count = 0
		self.bit_collect , ini = self.recover_data(self.bit_collect)
		
		if (ini):
			self.is_interrupt = True
			self.last_comm = time.time()
		return ini
	
	def terminal_input(self):
		while (self.take_input):
			self.rawdata_snd.append(raw_input('>>\n'))
			time.sleep(0.1)

	def terminal_output(self,str_):
		print '<<\n',str_,'>>\n'

	def s_ndis(self):
		if (self.is_interrupt):
			if (self.diac):
				self.cntrl_m = 1
				if (self.instant_write()):
					self.next_state = 1
					self.w_tck = True
				else:
					self.next_state = 2
					self.w_ack = True
					self.w_ret = True
		else:
			self.next_state = 0
			self.cntrl_m = 2
			self.instant_write()
			self.w_dck = True

	def s_dtor(self):
		if (self.is_interrupt):
			if (self.tack):
				self.next_state = 3
				self.cntrl_m = 7
				self.instant_write()
				self.w_tor = True
				self.w_dat = True
		else:
			self.next_state = 1
			self.cntrl_m = 6
			self.instant_write()
			self.w_tck = True

	def s_pkts(self):
		if (self.is_interrupt):
			if (self.ackn):
				self.string_data_ptr += self.bit_samples * 64
				self.cntrl_m =1
				if (self.instant_write()):
					self.next_state = 1
					self.w_tck = True
				else:
					self.next_state = 2
					self.w_ret = True
					self.w_ack = True
			elif (self.retr):
				self.next_state = 2
				self.cntrl_m = 1
				self.instant_write()
				self.w_ret = True
				self.w_ack = True				
		else:
			self.next_state = 2
			self.cntrl_m = 1
			self.instant_write()
			self.w_ret = True
			self.w_ack = True

	def s_ta1k(self):
		if (self.is_interrupt):
			if (self.dack):
				self.next_state = 5
				self.cntrl_m = 4
				self.instant_write()
				self.w_dat = True
				self.w_tor = True
			elif(self.dret):
				self.next_state = 5
				self.cntrl_m = 5
				self.instant_write()
				self.w_dat = True
			elif(self.tokr):
				self.next_state = 4
				self.cntrl_m = 7
				self.instant_write()
				self.w_tck = True
		else:
			self.next_state = 3
			self.cntrl_m = 7
			self.instant_write()
			self.w_dat = True
			self.w_tor = True

	def s_ta2k(self):
		if (self.is_interrupt):
			if (self.tack):
				self.cntrl_m = 1
				if (self.instant_write()):
					self.next_state = 1
					self.w_tck = True
				else:
					self.next_state = 2
					self.w_ack = True
					self.w_ret = True
		else:
			self.next_state = 4
			self.cntrl_m = 7
			self.instant_write()
			self.w_tck = True

	def s_pktr(self):
		if (self.is_interrupt):
			if (self.dack):
				self.next_state = 5
				self.cntrl_m = 4
				self.instant_write()
				self.w_dat = True
				self.w_tor = True
			elif(self.dret):
				self.next_state = 5
				self.cntrl_m = 5
				self.instant_write()
				self.w_dat = True
			elif(self.tokr):
				self.next_state = 4
				self.cntrl_m = 7
				self.instant_write()
				self.w_tck = True
		else:
			self.next_state = 5
			self.cntrl_m = 5
			self.instant_write()
			self.w_dat = True
			self.w_tor = True

	def instant_write(self):
		no_data = False
		if (self.cntrl_m == 1):
			if (self.string_data_ptr < len(self.string_data)):	#self.string_data_ptr update at ack +=(self.bit_samples * 64)
				self.instant_transmit = np.repeat([ self.string_data[self.string_data_ptr :self.string_data_ptr + (self.bit_samples * 64)]],5,axis=0).reshape(-1)
			else:
				if (self.rawdata_snd_ptr < len(self.rawdata_snd)): 
					self.string_data = self.rawtopack(self.rawdata_snd[self.rawdata_snd_ptr])
					self.rawdata_snd_ptr += 1
					self.string_data_ptr = 0
					self.instant_transmit = np.repeat([ self.string_data[self.string_data_ptr :self.string_data_ptr + (self.bit_samples * 64)]],5,axis=0).reshape(-1)
				else:
					self.string_data = []
					self.string_data_ptr = 0
					self.instant_transmit = np.repeat([self.control_pkt(6)],10,axis=0).reshape(-1) #token release
					no_data = True
		else:
			self.instant_transmit = np.repeat([self.control_pkt(self.cntrl_m)],10,axis=0).reshape(-1)
		return no_data

	def cntrl(self):
		if ((time.time() > self.last_comm + self.no_listen) and self.D_bit):
			#reset
			self.data_rec = ''
			self.last_pkt = ''
			self.is_interrupt = False
			self.D_bit = False
			self.current_state = 0 #one system at 0 and other at 1
			self.next_state = 0
			self.string_data_ptr = 0
			self.idle_time = self.dis_listen[self.dis_listen_p]
			self.dis_listen_p = (self.dis_listen_p + 1) % 3

		self.w_dat = False
		self.w_dis = False
		self.w_dck = False
		self.w_ack = False
		self.w_ret = False
		self.w_tor = False
		self.w_tck = False

		self.states[self.current_state]()
		self.is_interrupt = False
	
		self.dack = False
		self.dret = False
		self.disc = False
		self.diac = False
		self.ackn = False
		self.retr = False
		self.tokr = False
		self.tack = False

		self.current_state = self.next_state

		if (self.current_state == 0):
			self.D_bit = False
			self.idle_time = self.dis_listen[self.dis_listen_p]
			self.dis_listen_p = (self.dis_listen_p + 1) % 3	
		else:
			self.D_bit = True
			self.idle_time = self.cntrl_r_pkt_listen
		print(self.current_state)

	def AGC_control(self,max_val):
		if (max_val > 0.8):
			self.AGC_out -= 1000
		elif (max_val < 0.8 and self.AGC_out < 65535):
			self.AGC_out += 1000
		else:
			pass
		os.system("pacmd set-source-volume "+ self.command_finder_1[-1][0] +" "+ str(self.AGC_out))

	def session_(self,w_mode):
		self.new_point = True
		self.work_mode = w_mode
		len_buffer_val = self.buf_len_R
		if (w_mode):
			len_buffer_val = self.buf_len_T
		acoustic_stream = self.Digital_control.open(format=pyaudio.paFloat32,
			channels = 1,
			rate = self.fs,
			frames_per_buffer = len_buffer_val,
			input = not(w_mode),
			output = w_mode,
			stream_callback = self.callback,
			)
		while acoustic_stream.is_active():
			time.sleep(0.1)
			pass
		acoustic_stream.stop_stream()
		acoustic_stream.close()

	def initiate(self):
		input_thread = threading.Thread(target=self.terminal_input)
		input_thread.start()
		self.command_finder_0 =  os.popen("pactl list short sinks").readlines()
		self.command_finder_1 =  os.popen("pactl list short sources").readlines()
		self.start_time = time.time()
		while (not(self.pow_switch)):
			self.cntrl()
			os.system("pacmd set-sink-port "+ self.command_finder_0[-1][0] +" analog-output-headphones")
			os.system("pacmd set-sink-volume "+ self.command_finder_0[-1][0] +" 65535")
			#transmitting mode
			self.session_(True)

			os.system("pacmd set-source-port "+ self.command_finder_1[-1][0] +" analog-input-headphone-mic")
			os.system("pacmd set-source-volume "+ self.command_finder_1[-1][0] + " " + str(self.AGC_out))
			self.initial_time = time.time()
			#receiving mode
			self.session_(False)
			os.system("pacmd set-source-port "+ self.command_finder_1[-1][0] +" analog-input-internal-mic")
			
			if (time.time() > self.start_time + self.timer):
				self.pow_switch = 1
				self.take_input = False
		self.Digital_control.terminate()
		self.recording.stop()

if __name__ == '__main__':

	comm_init = transceiver()
	comm_init.initiate()
	comm_init.terminal_output('System Closed!!! press enter to stop busy threads')