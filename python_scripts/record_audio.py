import subprocess
import sys

# Get CLI arguments
AUDIO_FILE = sys.argv[1]            # Output filename
RECORD_SECONDS = int(sys.argv[2])   # Duration
AUDIO_RATE = sys.argv[3]            # Sample rate
AUDIO_CHANNELS = sys.argv[4]        # Channels (e.g., "1")
CHUNK = sys.argv[5]                 # Not used with arecord, but kept for compatibility

# Use correct arecord format
AUDIO_FORMAT = "S16_LE"
AUDIO_DEVICE = "plughw:2,0"  # You can also make this an argument if needed

def record_audio():
    print(f"\n[Audio] Recording {RECORD_SECONDS} seconds at {AUDIO_RATE} Hz, {AUDIO_CHANNELS} channels...")

    # Build the arecord command
    command = [
        "arecord",
        "-D", AUDIO_DEVICE,
        "-f", AUDIO_FORMAT,
        "-c", AUDIO_CHANNELS,
        "-r", AUDIO_RATE,
        "-d", str(RECORD_SECONDS),
        AUDIO_FILE
    ]

    try:
        subprocess.run(command, check=True)
        print(f"[Audio] Recording complete. File saved as: {AUDIO_FILE}")
    except subprocess.CalledProcessError as e:
        print(f"[Error] Recording failed: {e}")

if __name__ == "__main__":
    record_audio()