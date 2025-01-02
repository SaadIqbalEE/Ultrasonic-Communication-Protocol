#libs
import numpy as np
import pyaudio
import time
import threading
import os
from matplotlib import pyplot as plt

#transmitter class
class transmitter:
	#initializing variables
	def __init__(self):
		self.CRC_poly = np.uint8(220)	#CRC key
		self.preamble = np.uint8(170)	#packet initial/preamble
		self.bytesperpacket = 5			#char in packet
		self.fs = 48000					#Sampling frequency
		self.f0 = 18800					#F_0 = 18.8kHz
		self.f1 = 19200					#F_1 = 19.2khz
		self.bit_samples = 240
		self.D1 = np.cos(2*np.pi*(self.f0/np.float32(self.fs))*np.arange(0,self.bit_samples,1),dtype=np.float32)
		self.D2 = np.cos(2*np.pi*(self.f1/np.float32(self.fs))*np.arange(0,self.bit_samples,1),dtype=np.float32)
		
		self.raw_snd = []
		self.raw_snd_r = 0
		self.take_input = True
		
		self.out_ = []
		self.D_pointer = 0
		
		self.switch = 0
		self.timer = 20
		self.pow_timer = 600
		self.pow_switch = True


	#############CRC_calculator###################
	#return CRC of given 6 byte data
	def CRC_T(self,word):
		dividend = np.uint8(word[0])
		num_append = np.uint8(0)
		bits_values =np.append(word[1:6],[num_append],axis=0)
		temp_bits = np.unpackbits(bits_values)
		for i in range (0,len(temp_bits)):
			if(dividend>127): #10000000
				dividend = dividend^self.CRC_poly
			dividend = dividend<<1
			if(temp_bits[i]==1):
				dividend +=np.uint8(1)
		return dividend

	###encoding_packeting
	def rawtopack(self,string_data):
		f_pointer = 0
		f_length = len(string_data)
		modulo_size = (f_length + 1) % self.bytesperpacket
		num_null = 0 if modulo_size == 0 else self.bytesperpacket-(modulo_size)
		pack_size = ((f_length + 1 + num_null)/ self.bytesperpacket) + 3
		raw_pack = np.zeros(pack_size * (self.bytesperpacket + 5),dtype=np.uint8)
		packet_count = 0		
		for j in range (1, pack_size - 1):
			temp = (j*10) + 1
			raw_pack[temp] = self.preamble
			raw_pack[temp+1] = packet_count
			temp += 2
			for i in range(0,self.bytesperpacket):
				if (f_pointer < f_length):
					raw_pack[temp+i] = ord(string_data[f_pointer])
					f_pointer += 1
				elif (j <= pack_size - 3):
					if (i == self.bytesperpacket - 1):
						raw_pack[temp+i] = ord('\n')
					else:
						raw_pack[temp+i] = ord(' ')
				elif (j > pack_size - 3):
					raw_pack[temp+i] = ord('#')
				raw_pack[temp+5] = self.CRC_T(raw_pack[temp-1:temp+5])
			packet_count +=1
		return self.modulation_cpfsk(np.unpackbits(raw_pack))	

	##mod m = 2
	def modulation_cpfsk(self,mod_data):
		transmit_data = np.zeros(len(mod_data)*self.bit_samples,dtype=np.float32)
		for i in range(0,len(mod_data)):
			if(mod_data[i]):
				transmit_data[i*self.bit_samples:(i+1)*self.bit_samples]=self.D2
			else:
				transmit_data[i*self.bit_samples:(i+1)*self.bit_samples]=self.D1
		return transmit_data

	def callback(self,in_data, frame_count, time_info, status):
		port_status = self.switch
		data_out = np.zeros(frame_count, dtype=np.float32)
		if (len(self.out_) > 0):
			for i in range (0,frame_count):
				data_out[i] = self.out_[self.D_pointer]
				self.D_pointer = (self.D_pointer + 1) % len(self.out_)
		return (data_out, port_status)

	def terminal_input(self):
		while (self.take_input):
			self.raw_snd.append(raw_input('>>\n'))
			time.sleep(0.1)	

	def prog(self):
		digital_out = pyaudio.PyAudio()
		command_finder =  os.popen("pactl list short sinks").readlines()
		input_thread = threading.Thread(target=self.terminal_input)
		input_thread.start()
		script_initiate_time = time.time()

		while(self.pow_switch):
			if (time.time() > script_initiate_time + self.pow_timer):
				self.pow_switch = False
			if (self.raw_snd_r < len(self.raw_snd)):
				os.system("pacmd set-sink-port "+ command_finder[-1][0] +" analog-output-headphones")
				os.system("pacmd set-sink-volume "+ command_finder[-1][0] +" 65535")
				data = self.raw_snd[self.raw_snd_r]
				self.raw_snd_r += 1
				self.D_pointer = 0
				self.out_ = self.rawtopack(data)
				##transmit
				initial_time = time.time()
				self.switch = 0
				out_audio = digital_out.open(format=pyaudio.paFloat32,
					channels=1,
					rate=self.fs,	
					frames_per_buffer=self.fs/2,
					input=False,
					output=True,
					stream_callback=self.callback,
				)
				while out_audio.is_active():
					if (time.time() > initial_time + self.timer):
						self.switch = 1
					time.sleep(0.1)
					pass
				out_audio.stop_stream()
				out_audio.close()
			else:
				time.sleep(2)
		self.take_input = False
		digital_out.terminate()

if __name__ == '__main__':
	transmitting = transmitter()
	transmitting.prog()
	print('System Closed!!! press enter to stop busy threads')