# test_bark.py
import sounddevice as sd
import time
from VoskTest import BarkHandler  # assuming your BarkHandler is in VoskTest.py

def main():
    bh = BarkHandler(device=1)
    print("BarkHandler test. Press ENTER to play bark.")

    try:
        while True:
            input("Press ENTER to play bark...")
            print("Playing bark...")
            # play and block until done
            sd.play(bh.bark_data, bh.bark_fs, device=bh.device)
            sd.wait()  # wait until playback finishes
            print("Playback finished.")

    except KeyboardInterrupt:
        print("\nTest ended.")

if __name__ == "__main__":
    main()

