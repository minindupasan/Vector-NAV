import sounddevice as sd
import numpy as np
import time

sr = 24000
duration = 1.0
t = np.linspace(0, duration, int(sr * duration), False)
note = np.sin(440 * t * 2 * np.pi).astype(np.float32)

print("Opening stream at 24000 Mono...")
try:
    with sd.OutputStream(samplerate=24000, channels=1, dtype='float32', latency='high') as stream:
        for _ in range(5):
            stream.write(note[:24000]) # 1 second at a time
            print("Wrote 1s")
except Exception as e:
    print(f"Error: {e}")
