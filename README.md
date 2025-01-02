# Ultrasonic-Communication-Protocol
Data Leakage from Air-Gapped Systems over Ultrasonic Waves

---

# Uni-directional and Bi-directional Acoustic Communication

This repository contains implementations of acoustic communication systems, including uni-directional, bi-directional, and passive bi-directional modes. The systems leverage CPFSK modulation and operate using specific configurations for robust data transmission.

---

## Uni-directional Acoustic Communication

### Features:
- Converts message data to '.wav' format using 'data2wav'.
- Supports sending and receiving messages/commands with reliable alignment and sensitivity.

### File Descriptions:
1. **data2wav**: Converts message data into a '.wav' file (48 kHz).
2. **Transmitter_V0.1**: Sends messages/commands from a '.txt' file repeatedly until stopped (48 kHz).
3. **Transmitter_V1.1**: Sends messages/commands entered via terminal repeatedly until stopped (48 kHz).
4. **Receiver_V0.1**: Receives messages/commands, prints to the terminal, and appends to a file (48 kHz).
5. **Receiver_V1.1**: Enhanced version of the receiver with additional features for data handling (48 kHz).

### Notes:
- **Modulation**: m = 2, Bandwidth (BW) = 400 Hz, Carrier Frequency (Fc) = 19 kHz.
- **Key Features**:
  - Packet initial alignment.
  - CPFSK zero-crossing-based receiver.
  - Volume: 100%.
  - Sensitivity: Automatic Gain Control (AGC).
  - Line-of-sight (LOS) considerations.

---

## Bi-directional Acoustic Communication

### Features:
- Facilitates real-time communication using internal microphones and external speakers.

### File Descriptions:
1. **transceiver_client_V0.1**: Client-side implementation for bi-directional communication.
2. **transceiver_server_V0.1**: Server-side implementation for bi-directional communication.

---

## Passive Bi-directional Acoustic Communication

### Features:
- Utilizes port retasking to alternate between microphone and speaker roles.
- Requires passive sound components connected to the audio port of the PC/laptop.

### File Descriptions:
1. **ptransceiver_client_V0.1**: Client-side implementation for passive bi-directional communication.
2. **ptransceiver_server_V0.1**: Server-side implementation for passive bi-directional communication.

---

## Getting Started

### Prerequisites:
- PC/laptop with an audio port.
- External speakers and/or passive sound components.
- Internal or external microphone.

### Usage:
1. Clone the repository:  
   '''bash
   git clone https://github.com/SaadIqbalEE/Ultrasonic-Communication-Protocol.git
   cd acoustic-communication
   '''
2. Compile and run the desired transmitter, receiver, or transceiver program.

### Example:
For uni-directional transmission with 'Transmitter_V1.1':
'''bash
python Transmitter_V1.1.py
'''

---

## Contributing
Contributions are welcome! Feel free to submit issues or pull requests to enhance the functionality or documentation.

---

## License
This project is licensed under the MIT License. See the 'LICENSE' file for details.

--- 
