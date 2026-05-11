import sounddevice as sd
import numpy as np
import time

sr = 24000
duration = 1.0
t = np.linspace(0, duration, int(sr * duration), False)
# Generate a 440 Hz sine wave
note = np.sin(440 * t * 2 * np.pi).astype(np.float32)

print("Playing 24kHz Mono...")
sd.play(note, samplerate=sr, blocking=True)

print("Playing 48kHz Stereo with repeat...")
note_48 = np.repeat(note, 2)
stereo_48 = np.column_stack((note_48, note_48))
sd.play(stereo_48, samplerate=48000, blocking=True)
