import sounddevice as sd
import soundfile as sf

print(sd.query_devices())

device_index = 1  # whatever index your webcam mic has
samplerate = 44100

print("Recording...")
audio = sd.rec(frames=44100*5,
               samplerate=samplerate,
               channels=1,
               device=device_index)

sd.wait()
sf.write("webcam_audio.wav", audio, samplerate)
print("saved")