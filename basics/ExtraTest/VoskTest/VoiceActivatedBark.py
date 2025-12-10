# test_vosk_voice.py
import time
from SoundHandler import VoskKeywordProcessor  # your classes



def main():
    model_path = "./vosk-model-small-en-us-0.15"
    processor = VoskKeywordProcessor(model_path, audioIn=2, audioOut=1)

    print("=== Vosk Voice Processor Test ===")
    print("Speak into your microphone. Press Ctrl+C to stop.\n")

    # Start Vosk recognition
    processor.startVosk()

    try:
        while True:
            # Process queued actions
            processor.process()

            time.sleep(0.5)  # reduce CPU usage

    except KeyboardInterrupt:
        print("\nStopping test...")

    finally:
        processor.stopVosk()
        print("Test finished.")

if __name__ == "__main__":
    main()
