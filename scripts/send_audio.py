import requests
import sounddevice as sd
import numpy as np
import wave
import time

# Function to record audio from the microphone
def record_audio(filename, duration=4, samplerate=16000):
    print("Recording...")
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.float32)
    sd.wait()  # Wait until the recording is finished
    print("Recording complete.")
    
    # Save the recording to a WAV file
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)  # Mono audio
        wf.setsampwidth(2)  # 16-bit audio
        wf.setframerate(samplerate)
        wf.writeframes((audio * 32767).astype(np.int16).tobytes())
def check_robot_state(state_url):
    print("Checking robot state...")
    while True:
        try:
            response = requests.get(state_url)
            if response.status_code == 200:
                state = response.json()
                print("Robot busy state:", state['busy'])
                if not state['busy']:
                    print("Robot is no longer busy.")
                    break
            else:
                print("Error checking state:", response.status_code)
        except Exception as e:
            print("Failed to get state:", e)
        time.sleep(1)

# Function to send audio file to the server
def send_audio(file_path):
    url = "http://192.168.31.142:5000/act"  # Replace with actual IP of the Ubuntu machine
    state_url = "http://192.168.31.142:5000/state"
    files = {'file': open(file_path, 'rb')}

    try:
        response = requests.post(url, files=files)
        if response.status_code == 200:
            print("Response:", response.json())
            check_robot_state(state_url)
        else:
            print("Error:", response.status_code, response.text)
    except Exception as e:
        print("Failed to send audio:", e)

# Main execution
if __name__ == "__main__":
    audio_file = 'temp_recording.wav'  # Filename to store the recording

    # Record audio from the microphone
    record_audio(audio_file)

    # Send the recorded audio to the server
    send_audio(audio_file)
