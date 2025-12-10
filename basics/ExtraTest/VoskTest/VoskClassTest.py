# test_vosk_voice.py
import time
from SoundHandler import VoskKeywordProcessor  # your classes

def main():
    model_path = "./vosk-model-small-en-us-0.15"

    # Initialize processor
    processor = VoskKeywordProcessor(model_path, device=2)

    print("=== Vosk Voice Processor Test ===")
    print("Speak into your microphone. Press Ctrl+C to stop.\n")

    # Start Vosk recognition
    processor.startVosk()

    try:
        while True:
            # Process queued actions
            action = processor.getAction()
            while action is not None:
                print(f"Action detected: {action}")
                # Optionally, do something like play bark here
                action = processor.getAction()

            # Only print last recognized text if a sentence triggered an action
            if processor.hasNewSentence():
                print(f"Last recognized text: '{processor.getLastRecognizedText()}'")
                # Reset flag so it's only printed once
                processor.voskSentenceDone = False

            time.sleep(0.5)  # reduce CPU usage

    except KeyboardInterrupt:
        print("\nStopping test...")

    finally:
        processor.stopVosk()
        print("Test finished.")

if __name__ == "__main__":
    main()
