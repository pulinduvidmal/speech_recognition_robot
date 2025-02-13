# speech_recognition_robot

## Overview
This project implements a speech-controlled UR10 robot using OpenAI's Whisper model for speech recognition. The robot listens to spoken commands and assembles vehicles accordingly.

![image](https://github.com/user-attachments/assets/52e186d1-a221-4aac-a293-f130d5640bb6)

## Features
- **Speech Recognition**: Utilizes OpenAI's Whisper model for accurate speech-to-text conversion.
- **Robot Control**: Controls the UR10 robot using the URBasic library.
- **Flask Server**: A Flask-based server processes audio inputs and sends commands to the robot.

## Setup Instructions

### 1. Install Dependencies
Ensure you have Python 3.8+ installed, then install the required libraries:
```bash
pip install -r requirements.txt
```

### 2. Run the Flask Server
Start the server to process speech and control the UR10 robot:
```bash
python scripts/server.py
```

### 3. Record and Send Audio
Run the send_audio.py script to record and send speech commands:
```bash
python scripts/send_audio.py
```
## Dependencies
- `torch`
- `transformers`
- `flask`
- `numpy`
- `sounddevice`
- `URBasic`
- `whisper`

## Future Improvements
- Improve recognition accuracy with more training data.
- Support additional languages.







