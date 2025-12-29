#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
import time
import sys
import termios
import tty
import select
from scipy.signal import butter, filtfilt

# RPi optimization
import os
os.environ["OMP_NUM_THREADS"] = "4"
os.environ["CT2_FAST_NATIVE_THREADING"] = "1"

class TextInputNode(Node):
    def __init__(self):
        super().__init__('text_input_node')
        self.publisher_ = self.create_publisher(String, '/text', 10)
        
        # CONFIG
        self.SAMPLE_RATE = 16000
        self.BLOCK_DURATION = 0.1
        self.CHANNELS = 1
        self.FRAMES_PER_BLOCK = int(self.SAMPLE_RATE * self.BLOCK_DURATION)
        sd.default.samplerate = self.SAMPLE_RATE
        sd.default.channels = self.CHANNELS
        
        self.model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
        
        # STATE
        self.recording_active = threading.Event()
        self.audio_data = []
        self.audio_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.input_mode = 0  # 0=setup, 1=voice, 2=keyboard
        self.current_text = ""
        
    def preprocess_audio(self, audio_array):
        b, a = butter(4, 300, btype='high', fs=self.SAMPLE_RATE)
        return filtfilt(b, a, audio_array)
    
    def initial_setup(self):
        """Ask initial mode choice"""
        print("\n=== TEXT INPUT SELECTOR ===")
        print("1: 🎤 Voice input")
        print("2: ⌨️  Keyboard input")
        
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        try:
            while self.input_mode == 0:
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == '1':
                        self.input_mode = 1
                        print("\n✅ Voice mode selected! ESC to switch")
                        break
                    elif key == '2':
                        self.input_mode = 2
                        print("\n✅ Keyboard mode selected! ESC to switch")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def keyboard_thread(self):
        """Main keyboard handler with mode switching"""
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        print(f"\nMode: {'🎤 VOICE' if self.input_mode==1 else '⌨️ KEYBOARD'} | ESC=Toggle | Ctrl+C=Quit")
        
        try:
            while not self.stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    # ESC - Toggle mode
                    if key == '\x1b':
                        self.input_mode = 3 - self.input_mode  # 1↔2
                        self.current_text = ""
                        print(f"\n🔄 Switched to {'🎤 VOICE' if self.input_mode==1 else '⌨️ KEYBOARD'} mode")
                        continue
                    
                    if self.input_mode == 1:  # VOICE MODE
                        if key == ' ':
                            print("\n🎤 Recording... (ENTER to stop)")
                            self.recording_active.set()
                            with self.audio_lock:
                                self.audio_data.clear()
                        elif key == '\r' and self.recording_active.is_set():
                            print("\n⏹️  Processing...")
                            self.recording_active.clear()
                            self.process_recording()
                            
                    else:  # KEYBOARD MODE
                        if key == '\r':  # ENTER - publish
                            if self.current_text.strip():
                                text = self.current_text.strip()
                                print(f"\n✅ Published: '{text}'")
                                self.publish_text(text)
                                self.current_text = ""
                        elif key == '\x7f':  # Backspace
                            self.current_text = self.current_text[:-1]
                            sys.stdout.write('\b \b')
                            sys.stdout.flush()
                        else:
                            self.current_text += key
                            sys.stdout.write(key)
                            sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def audio_callback(self, indata, frames, time_, status):
        if status:
            print(status)
        if self.recording_active.is_set():
            data = indata[:, 0].astype(np.float32)
            with self.audio_lock:
                self.audio_data.extend(data)
    
    def process_recording(self):
        with self.audio_lock:
            if len(self.audio_data) < self.SAMPLE_RATE * 0.5:
                print("❌ Audio too short")
                return
            audio_array = np.array(self.audio_data[-self.SAMPLE_RATE*3:], dtype=np.float32)
        
        # ENHANCE + NORMALIZE
        audio_array = self.preprocess_audio(audio_array)
        audio_array = audio_array.astype(np.float32)
        audio_array = np.clip(audio_array, -1.0, 1.0)
        if np.max(np.abs(audio_array)) > 0:
            audio_array /= np.max(np.abs(audio_array))
        
        print("🔮 Transcribing...")
        segments, _ = self.model.transcribe(audio_array, language="en", vad_filter=True)
        text = " ".join(seg.text.strip() for seg in segments).strip()
        
        if text:
            print(f"✅ Voice: '{text}'")
            self.publish_text(text)
        else:
            print("❌ No speech detected")
    
    def publish_text(self, text):
        """Publish to ROS2 /text topic"""
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{text}"')
    
    def audio_thread(self):
        with sd.InputStream(
            samplerate=self.SAMPLE_RATE,
            channels=self.CHANNELS,
            blocksize=self.FRAMES_PER_BLOCK,
            dtype="float32",
            callback=self.audio_callback,
        ):
            while not self.stop_event.is_set():
                time.sleep(0.1)
    
    def run(self):
        """Main execution"""
        self.initial_setup()
        
        # Start threads
        kb_thread = threading.Thread(target=self.keyboard_thread, daemon=True)
        kb_thread.start()
        
        audio_t = threading.Thread(target=self.audio_thread, daemon=True)
        audio_t.start()
        
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TextInputNode()
    node.run()

if __name__ == '__main__':
    main()
