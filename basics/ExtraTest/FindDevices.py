import sounddevice as sd

def list_audio_devices():
    print("Available audio devices:\n")
    devices = sd.query_devices()
    for i, dev in enumerate(devices):
        print(f"{i}: {dev['name']}")
        print(f"    Input channels : {dev['max_input_channels']}")
        print(f"    Output channels: {dev['max_output_channels']}")
        print(f"    Default samplerate: {dev['default_samplerate']}")
        print(f"    Host API: {sd.query_hostapis(dev['hostapi'])['name']}\n")

if __name__ == "__main__":
    list_audio_devices()

