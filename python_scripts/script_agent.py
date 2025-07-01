import os
import re
import json
import time
import requests
import subprocess
import serial
import time
import RPi.GPIO as GPIO

# For camera
from picamera2 import Picamera2

# Import the OpenAI Library
from openai import OpenAI

#############################################
# 1) SETUP & CONFIG
#############################################

# Adjustment values for the environment
WHISPER_BIN = "./whisper.cpp/build/bin/whisper-cli"     # Path to the whisper.cpp CLI
WHISPER_MODEL = "whisper.cpp/models/ggml-small.en.bin"  # whisper.cpp model file
AUDIO_FILE = "command.wav"                              # Audio file of the recording
IMAGE_FILE = "robot_view.jpg"                           # The picture taken from the camera

# Recording parameters
RECORD_SECONDS = 10
AUDIO_RATE = 44100
AUDIO_CHANNELS = 1
CHUNK = 1024

# TTS  parameters
volume = 1000   # Amplitude
speed = 175     # Speed
pitch = 40      # Pitch

# Arduino serial port configuration
ARDUINO_PORT = '/dev/arduino'   # /dev/arduino -> ttyACM0
ARDUINO_BAUDRATE = 115200

# Option to choose between the Whisper API or the local whisper.cpp CLI
USE_API_WHISPER = True

# OpenAI/GPT initialization
client = OpenAI(
    api_key="your_key"  # Replace with your actual API key
)

assistant_id="assistant_id"    # The assistant ID to use
thread_id="thread_script"                       # The thread ID

# Pin setup for the ultrasonic sensor
TRIG_LEFT = 23
ECHO_LEFT = 24
TRIG_RIGHT = 27
ECHO_RIGHT = 22

#############################################
# 2) FUNCTION TO RECORD AUDIO
#############################################
def record_audio(duration=RECORD_SECONDS, output_file=AUDIO_FILE, rate=AUDIO_RATE, channels=AUDIO_CHANNELS, chunk=CHUNK):
    """
    Record audio by calling an external Python script with sudo privileges.
    """
    print(f"[System] Calling audio recording script with sudo...")
    
    # Command to call the external script with sudo
    subprocess.run(['sudo', 'python3', 'record_audio.py', output_file, str(duration), str(rate), str(channels), str(chunk)], check=True)

#############################################
# 3) FUNCTION TO RUN WHISPER TRANSCRIPTION
#############################################
def transcribe_with_whisper(wav_file=AUDIO_FILE):
    """
    Use the whisper.cpp CLI tool to transcribe the given wav file.
    Returns the transcribed text (str).
    If Whisper produces no text, return an empty string.
    """
    if not USE_API_WHISPER:
        # Use the local Whisper to transcribe the audio
        if not os.path.exists(wav_file):
            print(f"[Whisper] WAV file not found: {wav_file}")
            return ""

        if not os.path.exists(WHISPER_BIN):
            print(f"[Whisper] Whisper CLI not found at: {WHISPER_BIN}")
            return ""

        cmd = [
            WHISPER_BIN,
            "-m", WHISPER_MODEL,
            "-f", wav_file,
            # optional flags if needed, e.g. language, etc.
        ]

        print("[Whisper] Running whisper.cpp CLI to transcribe audio...")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f"[Whisper] Error during transcription: {result.stderr}")
                return ""
            # The transcript is often at the end of stdout, but this can vary by version
            output_lines = result.stdout.strip().split("\n")
            if len(output_lines) == 0:
                return ""
            # Usually the final line is the recognized text
            transcript = output_lines[-1].strip()
            print(f"[Whisper] Transcription: {transcript}")
            return transcript
        except Exception as e:
            print(f"[Whisper] Exception running whisper CLI: {e}")
            return ""
    else:
        transcript = ""
        # Use the API to transcribe the audio
        print("[Whisper] Transcribing audio with API...")
        try:
            # Open the audio file
            audio_file = open(wav_file, "rb")
        except FileNotFoundError:
            print(f"[Whisper] WAV file not found: {wav_file}")
        try:
            # Call the API to transcribe the audio
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )
        except Exception as e:
            print(f"[Whisper] Error during transcription: {e}")

        return transcript.text

#############################################
# 4) FUNCTION TO CAPTURE AN IMAGE
#############################################
def capture_image(output_file=IMAGE_FILE):
    """
    Capture an image from the Pi camera using libcamera and save to `output_file`.
    """
    # Run the libcamera-jpeg command
    print("[Camera] Running the libcamera command...")
    subprocess.run(["libcamera-jpeg", "-o", output_file, "--timeout", "0.1"])

    print(f"[Camera] Image saved as {output_file}")

#############################################
# 5) FUNCTION TO CALL GPT API
#############################################
def call_gpt(environment_data, user_instruction, thread_id):
    """
    Send the environment data, user instruction, and image URL
    to the GPT model, parse the JSON result, and return it.
    """
    print("[GPT] Sending data to the GPT API...")

    # Construct final user prompt
    user_prompt = f"""
    Environment data:
    "{environment_data}"

    User instruction:
    "{user_instruction}"
    """

    try:
        print("[GPT] Trying to upload the image file...")
        # Upload the image to the assistant
        file_response = client.files.create(
            file=open(IMAGE_FILE, "rb"),
            purpose="vision"
        )

        image_file_id = file_response.id # Get the file ID

        # Send the user prompt to the assistant
        message = client.beta.threads.messages.create(
            thread_id=thread_id,
            role="user",
            content=[
                {
                    "type": "text",
                    "text": user_prompt
                },
                {
                    "type": "image_file",
                    "image_file": {"file_id": image_file_id}
                }
            ]
        )

        # Run the assistant
        run = client.beta.threads.runs.create(
            thread_id=thread_id,
            assistant_id=assistant_id
        )

        # Wait for the assistant to finish processing
        while True:
            run_status = client.beta.threads.runs.retrieve(thread_id=thread_id, run_id=run.id)
            if run_status.status == "completed":
                break
            time.sleep(2)  # Wait before checking again

        # Get the assistant's reply
        assistant_reply = client.beta.threads.messages.list(thread_id=thread_id)

        print("Raw Model Response:")
        print(assistant_reply)

        # Get the assistant's messages and parse them
        assistant_messages = [msg for msg in assistant_reply.data if msg.role == "assistant"]
        if not assistant_messages:
            print("[GPT] No assistant messages found.")
            return None

        # Take the last (most recent) assistant message
        last_assistant_msg = assistant_messages[0]
        json_string = None
        for block in last_assistant_msg.content:
            if block.type == "text":
                json_string = block.text.value
                break

        if not json_string:
            print("[GPT] No text block found in assistant's message.")
            return None

        # Attempt to parse the JSON
        data = json.loads(json_string)
        return data

    except json.JSONDecodeError as e:
        print("[GPT] JSON decode error:", e)
        print("Assistant's reply was not valid JSON.")
        return None
    except Exception as ex:
        print("[GPT] Error with OpenAI API call:", ex)
        return None

#############################################
# 7) FUNCTION TO OUTPUT TTS SYNCHRONOUSLY
#############################################
def speak(text):
    """
    Uses the speaker to output in speech the given text, using the blocking run
    """
    print("[TTS] Speaking...")
    subprocess.run(["espeak-ng", f"-a {volume}", f"-s {speed}", f"-p {pitch}", text])

#############################################
# 7) FUNCTION TO OUTPUT TTS ASYNCHRONOUSLY
#############################################
def speak_async(text):
    """
    Uses the speaker to output in speech the given text, using the non-blocking Popen
    """
    print("[TTS] Speaking...")
    subprocess.Popen(["espeak-ng", f"-a {volume}", f"-s {speed}", f"-p {pitch}", text])

#############################################
# 8) ARDUINO COMMS FUNCTIONS
#############################################
def init_arduino():
    """
    Initialize the Arduino serial connection.
    Returns True if successful, False otherwise.
    """
    global arduino_serial
    try:
        arduino_serial = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for the connection to be established
        print(f"[ARDUINO] Serial port {ARDUINO_PORT} initialized at {ARDUINO_BAUDRATE} baud.")
        return True
    except serial.SerialException as e:
        print(f"[ARDUINO] Error initializing serial port: {e}")
        arduino_serial = None
        return False

def send_arduino_direct_command(command):
    """
    Send a command to Arduino and wait for "Finished instruction." response.
    Returns True if command completed successfully, False otherwise.
    """
    global arduino_serial
    
    if arduino_serial is None:
        print("[ARDUINO] Serial port not initialized.")
        return False

    try:
        # Send command
        arduino_serial.write(command.encode())
        print(f"[ARDUINO] Sent command: {command}")

        # Wait for the command execution response - "Finished instruction."
        while True:
            response = arduino_serial.readline().decode().strip()
            if response:  # Only print non-empty responses
                print(f"[ARDUINO] Received response: {response}")
                
                # Check if the command has finished executing
                if "Finished instruction." in response:
                    print("[ARDUINO] Command execution completed.")
                    return True

    except serial.SerialException as e:
        print(f"[ARDUINO] Serial error: {e}")
        return False
    
def send_arduino_position_command(command, timeout):
    """
    Send a position command to Arduino and wait for "Finished instruction." response.
    Returns True if command completed successfully, False otherwise.
    """
    global arduino_serial

    if arduino_serial is None:
        print("[ARDUINO] Serial port not initialized.")
        return False

    try:
        # Send command
        arduino_serial.write(command.encode())
        print(f"[ARDUINO] Sent position command: {command}")

        # Wait for the command execution response - "Finished instruction."
        while True:
            response = arduino_serial.readline().decode().strip()
            if response:  # Only print non-empty responses
                print(f"[ARDUINO] Received response: {response}")

                # Check if the command has finished executing
                if "Finished instruction." in response:
                    print("[ARDUINO] Command execution completed.")
                    print("[ARDUINO] Waiting for the timeout.")
                    time.sleep(timeout)
                    return True

    except serial.SerialException as e:
        print(f"[ARDUINO] Serial error: {e}")
        return False

def send_arduino_commands(movements, steps_list):
    """
    Execute multiple Arduino commands one after the other.
    
    Args:
        movements (str): Comma-separated movement commands (e.g., "forward,left,sit")
        steps_list (str): Comma-separated steps for each movement (e.g., "2,5,0")
    
    Returns:
        bool: True if all commands executed successfully, False otherwise
    """
    if not movements or movements == "none":
        print("[ARDUINO] No movements to execute.")
        return True
    
    # Parse movements and steps
    movement_list = [m.strip() for m in movements.split(',')]
    steps_str_list = [s.strip() for s in steps_list.split(',')]
    
    # Convert steps to integers
    try:
        steps_int_list = [int(s) for s in steps_str_list]
    except ValueError as e:
        print(f"[ARDUINO] Error parsing steps: {e}")
        return False
    
    # Ensure movements and steps lists have the same length
    if len(movement_list) != len(steps_int_list):
        print(f"[ARDUINO] Mismatch: {len(movement_list)} movements but {len(steps_int_list)} steps.")
        return False
    
    print(f"[ARDUINO] Executing {len(movement_list)} commands sequentially...")

    # Execute each command sequentially
    for i, (movement, steps) in enumerate(zip(movement_list, steps_int_list)):
        print(f"[ARDUINO] Command {i+1}/{len(movement_list)}: {movement} with {steps} steps")

        # check if it is a position command: "movement": "1:30 2:80 5:100,2:60", "steps": "2,0.5"
        if re.match(r'^(?:\s*(?:1[0-5]|[0-9]):[0-9]{1,3}\s*){1,16}$', movement):
            # This is a position command
            print(f"[ARDUINO] Detected position command: {movement}")
            # Send the position command with a timeout of steps seconds
            command_success = send_arduino_position_command(movement, timeout=steps)
            if not command_success:
                print(f"[ARDUINO] Position command {i+1} failed: {movement}")
                return False
        else:
            # This is a regular movement command
            arduino_command = f"move,{movement},{steps}"
            
            # Send command and wait for completion
            command_success = send_arduino_direct_command(arduino_command)
            if not command_success:
                print(f"[ARDUINO] Command {i+1} failed: {arduino_command}")
                return False
        
        print(f"[ARDUINO] Command {i+1} completed successfully.")
        
        # Small delay between commands for safety
        time.sleep(0.5)
    
    print("[ARDUINO] All commands executed successfully.")
    return True

def close_arduino():
    """
    Close the Arduino serial connection.
    """
    global arduino_serial
    if arduino_serial and arduino_serial.is_open:
        arduino_serial.close()
        print("[ARDUINO] Serial port closed.")
        arduino_serial = None

#############################################
# 9) Ultrasonic Sensor Functions
#############################################
def setup_ultrasonic():
    print("[Ultrasonic] Setting up ultrasonic sensors pins.")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)

def read_distance(trig, echo):
    # Send 10us pulse to trigger
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    stop = time.time()

    # Wait for echo to go high
    while GPIO.input(echo) == 0:
        start = time.time()
    # Wait for echo to go low
    while GPIO.input(echo) == 1:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2  # cm
    return round(distance, 2)

def get_sensor_data():
    left = read_distance(TRIG_LEFT, ECHO_LEFT)
    right = read_distance(TRIG_RIGHT, ECHO_RIGHT)
    return left, right

#############################################
# 10) MAIN LOOP
#############################################
def main():
    print("[System] Robot control script started. Press Ctrl+C to exit.")

    speak_async("Waking up now.")

    # Configuring the camera, it needs to run without a timeout when it is first opened
    if os.path.exists(IMAGE_FILE):
        print(f"[System] Removing old image file: {IMAGE_FILE}")
        os.remove(IMAGE_FILE)
    subprocess.run(["libcamera-jpeg", "-o", IMAGE_FILE])

    # Initialize Arduino connection
    if not init_arduino():
        print("[System] Failed to initialize Arduino. Exiting.")
        return
    
    # Setup ultrasonic sensors
    try:
        setup_ultrasonic()
    except Exception as e:
        print(f"[System] Error setting up ultrasonic sensors: {e}")
        close_arduino()
        return
    print("[System] Ultrasonic sensors initialized.")

    # Creating a thread for the AI agent:
    try:
        thread = client.beta.threads.create()
        thread_id = thread.id
    except Exception as e:
        print(f"[System] Error creating thread: {e}")
        return

    while True:
        try:
            speak("Welcome, please give an instruction.")
            # --- 1) Record audio for 10 seconds ---
            record_audio(duration=RECORD_SECONDS, output_file=AUDIO_FILE)

            transcription = "" #"Hello robot, please use the direct position command to move flat. First move the front legs in the flat position, then after 1 second move the back legs as well. You're staring from the standing position. Make sure to set the values for all 12 servos."  # empty usually

            speak_async("Processing instruction.")
            # --- 2) Transcribe with Whisper ---
            transcription = transcribe_with_whisper(AUDIO_FILE)

            print(f"[System] The transcription is: {transcription}")

            # --- 3) Check if transcription is empty, if no speech found, set a default command ---
            if not transcription.strip():
                print("[System] No speech detected, defaulting to fallback instruction.")
                speak_async("No speech detected, acting on my own.")
                transcription = "No instructions detected, act on your own."
            else:
                speak_async(f"The instruction was translated as: {transcription}")

            # --- 4) Delete the old image file if it exists ---
            if os.path.exists(IMAGE_FILE):
                print(f"[System] Removing old image file: {IMAGE_FILE}")
                os.remove(IMAGE_FILE)

            # --- 5) Capture image ---
            capture_image(IMAGE_FILE)
            print(f"[System] Image captured and saved as {IMAGE_FILE}")

            # --- 6) Get sensor data ---
            left_dist, right_dist = get_sensor_data()
            # left_dist = 50  # Simulated distance for left sensor
            # right_dist = 50  # Simulated distance for right sensor
            environment_data = (
                f"Left sensor: {left_dist} cm (50° right-inclined), "
                f"Right sensor: {right_dist} cm (50° left-inclined)."
            )
            user_instruction = transcription

            result_json = None

            # --- 7) Call GPT with environment data & transcription ---
            result_json = call_gpt(environment_data, user_instruction, thread_id)
            if not result_json:
                print("[System] GPT returned no valid instructions.")
                speak_async("GPT returned no valid instructions, internal error.")
            else:
                # Access the fields
                movement = result_json.get("movement", "none")
                steps = result_json.get("steps", 0)
                speech = result_json.get("speech", "")

                # --- 7) Execute commands ---
                print("\nMovement Instructions:")
                print(f" Movement: {movement}")
                print(f" Steps: {steps}")
                print(f" TTS Output: {speech}")
                print("---------------------------------------------")

                speak_async(speech)

                # Send movement command to Arduino if movement is specified
                if movement != "none":
                    print(f"[System] Sending Arduino commands: {movement}, Steps: {steps}")
                    commands_success = send_arduino_commands(movement, str(steps))
                    if commands_success:
                        print("[System] Arduino commands completed successfully.")
                    else:
                        print("[System] Arduino commands failed.")
                        speak_async("Movement commands failed.")

            # --- 8) Entering the next loop ---
            print("\n[System] Ready for next instruction...")

            time.sleep(1)  # Small delay before the next iteration

            # Ending the script here
            # print("\n[System] Exiting loop. Goodbye!")
            # break

        except KeyboardInterrupt:
            print("\n[System] Exiting loop. Goodbye!")
            break
        except Exception as e:
            print(f"[System] Error in main loop: {e}")
            time.sleep(2)  # small delay to avoid rapid crash loop

if __name__ == "__main__":
    main()