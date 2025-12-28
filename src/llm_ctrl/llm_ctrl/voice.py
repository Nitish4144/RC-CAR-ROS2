import threading
import numpy as np
import sounddevice as sd
from pynput import keyboard
from faster_whisper import WhisperModel
import time
"""
for rpi optimization
import os
os.environ["OMP_NUM_THREADS"] = "4"  # RPi CPU optimization [web:84]
os.environ["CT2_FAST_NATIVE_THREADING"] = "1"

# Replace your model line:
model = WhisperModel("distil-small.en", device="cpu", compute_type="int8")  # 6x faster + accurate [web:97]
def preprocess_audio(audio_array):
    #Remove noise/rumble for +20% accuracy
    from scipy.signal import butter, filtfilt
    b, a = butter(4, 300, btype='high', fs=SAMPLE_RATE)  # Cut low rumble
    return filtfilt(b, a, audio_array)

"""
# ======================
# CONFIG
# ======================
SAMPLE_RATE = 16000
BLOCK_DURATION = 0.1
RECORD_DURATION = 3.0
CHANNELS = 1
FRAMES_PER_BLOCK = int(SAMPLE_RATE * BLOCK_DURATION)
RECORD_FRAMES = int(SAMPLE_RATE * RECORD_DURATION)

sd.default.samplerate = SAMPLE_RATE
sd.default.channels = CHANNELS

model = WhisperModel("distil-small.en", device="cpu", compute_type="int8")
LANGUAGE = "en"
BEAM_SIZE = 1

# ======================
# STATE
# ======================
recording_active = threading.Event()
audio_data = []
audio_lock = threading.Lock()
stop_event = threading.Event()
space_pressed = False  # Track spacebar state

# ======================
# KEYBOARD
# ======================
def on_press(key):
    global space_pressed
    if key == keyboard.Key.space and not space_pressed:
        space_pressed = True
        print("\n🎤 Recording... (release space to stop)")
        recording_active.set()
        with audio_lock:
            audio_data.clear()
    return True  # do NOT stop listener

def on_release(key):
    global space_pressed
    if key == keyboard.Key.space and space_pressed:
        space_pressed = False
        print(" Stopping recording...")
        recording_active.clear()
        process_recording()  # IMMEDIATE PROCESSING
        return True  # keep listener running

    if key == keyboard.Key.esc:
        stop_event.set()
        return False  # stop listener and exit
    return True

# ======================
# AUDIO CALLBACK
# ======================
def audio_callback(indata, frames, time_, status):
    if status:
        print(status)
    if recording_active.is_set():
        data = indata[:, 0].astype(np.float32)
        with audio_lock:
            audio_data.extend(data)

# ======================
# PROCESS RECORDING
# ======================
def process_recording():
    with audio_lock:
        if len(audio_data) < SAMPLE_RATE * 0.3:
            print(" Audio too short (<0.3s)")
            return

        arr = np.array(audio_data[:RECORD_FRAMES], dtype=np.float32)

    print(" Transcribing...")

    max_val = np.max(np.abs(arr))
    if max_val > 0:
        arr /= max_val

    segments, _ = model.transcribe(arr, language=LANGUAGE, beam_size=BEAM_SIZE)
    text = "".join(seg.text for seg in segments).strip()

    if text:
        print(f" You said: '{text}'")
    else:
        print(" No speech detected")



"""def process_recording():
    with audio_lock:
        if len(audio_data) < SAMPLE_RATE * 0.5:  # Need more audio for accuracy
            print(" Audio too short")
            return
        audio_array = np.array(audio_data[:RECORD_FRAMES], dtype=np.float32)
    
    # ENHANCE AUDIO
    audio_array = preprocess_audio(audio_array)
    
    # NORMALIZE
    max_val = np.max(np.abs(audio_array))
    if max_val > 0:
        audio_array /= max_val
    
    print(" High-accuracy transcription...")
    
    # SUPER ACCURATE TRANSCRIPTION
    segments, _ = model.transcribe(
        audio_array,
        language="en",
        temperature=0,      # Consistent results
        best_of=5,          # Pick BEST of 5 attempts
        beam_size=5,        # Smarter search
        vad_filter=True     # Auto-remove silence
    )
    
    text = " ".join(seg.text.strip() for seg in segments).strip()
    
    if text:
        print(f" '{text}'")  # YOUR RC CAR COMMAND!
        # PIPE TO OLLAMA/ROS2 HERE 
        #send_to_rc_car(text)
    else:
        print(" No speech")
"""
# ======================
# AUDIO THREAD
# ======================
def audio_thread():
    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        blocksize=FRAMES_PER_BLOCK,
        dtype="float32",
        callback=audio_callback,
    ):
        print(" Listening for SPACEBAR (ESC to quit)")
        while not stop_event.is_set():
            time.sleep(0.1)

# ======================
# MAIN
# ======================
if __name__ == "__main__":
    print("Spacebar PTT")
    print("• Hold SPACEBAR → record")
    print("• Release SPACEBAR → transcribe")
    print("• Press ESC → quit\n")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    audio_t = threading.Thread(target=audio_thread, daemon=True)
    audio_t.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    finally:
        stop_event.set()
        listener.stop()
        print(" Done!")
