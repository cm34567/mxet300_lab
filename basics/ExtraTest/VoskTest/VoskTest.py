from enum import Enum
import time
import threading
import queue
import json
import sounddevice as sd
import soundfile as sf
import numpy as np
import curses
from vosk import Model, KaldiRecognizer

class VoskRecognizer:
    @staticmethod
    def list_devices():
        """Prints a list of available audio devices with indices."""
        print(sd.query_devices())

    def __init__(self, model_path, samplerate=16000, blocksize=8000, device=None, on_partial=None, on_final=None):
        """
        model_path: path to Vosk model directory
        samplerate: audio sampling rate (Vosk expects 16000)
        blocksize: number of samples per audio chunk
        device: audio device index or name (optional)
        on_partial: callback for partial recognition text
        on_final: callback for final recognition text
        """
        self.model_path = model_path
        self.samplerate = samplerate
        self.blocksize = blocksize
        self.device = device
        self.on_partial = on_partial
        self.on_final = on_final

        self._audio_queue = queue.Queue()
        self._running = False

        # Load model once
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, samplerate)

        self._thread = None
        self._stream = None

    def _audio_callback(self, indata, frames, time, status):
        if status:
            print("Audio status:", status)
        self._audio_queue.put(bytes(indata))

    def _processing_thread(self):
        """Processes audio in background using Vosk."""
        while self._running:
            try:
                data = self._audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "")
                if text and self.on_final:
                    self.on_final(text)
            else:
                partial = json.loads(self.recognizer.PartialResult())
                text = partial.get("partial", "")
                if text and self.on_partial:
                    self.on_partial(text)

    def start(self):
        """Start recording + recognition thread."""
        if self._running:
            return

        self._running = True

        # Start processing thread
        self._thread = threading.Thread(target=self._processing_thread, daemon=True)
        self._thread.start()

        # Start audio stream
        self._stream = sd.RawInputStream(
            samplerate=self.samplerate,
            blocksize=self.blocksize,
            dtype='int16',
            channels=1,
            device=self.device,
            callback=self._audio_callback
        )
        self._stream.start()

        print(f"VoskRecognizer started (device={self.device}).")

    def stop(self):
        """Stop recording + shut down thread."""
        if not self._running:
            return

        self._running = False
        if self._stream:
            self._stream.stop()
            self._stream.close()
        if self._thread:
            self._thread.join()

        print("VoskRecognizer stopped.")



class VoskKeywordProcessor:

    class Actions(Enum):
        BARK = 0
        SPIN = 1
        STOP = 2

    def __init__(self, model_path, device = -1):
        self.actionQueue = queue.Queue()
        self.triggerKeywords = ["rex", "rec", "recs", "wrecks", "rats", "racks", "wracks"]
        self.actionKeywords = {
            self.Actions.SPIN: ["spin", "spun", "spine", "spoon"],
            self.Actions.BARK: ["bark", "mark", "park", "dark", "shark", "lark"],
            self.Actions.STOP: ["stop", "stoop", "step", "top", "stob", "stock"]
        }
        self.lastRecognizedText = ""
        self.sentenceTriggered = False

        self.vosk_rec = vosk_rec = VoskRecognizer(
            model_path,
            16000,
            16000,
            None,
            None,
            self.processText)

    def processText(self, text):
        self.lastRecognizedText = text
        self.sentenceTriggered = True
        # print(text)
        words = text.split()

        isKeyword = [x in self.triggerKeywords for x in words]
        if (any(isKeyword)):
            # print("Keyword detected")
            index = isKeyword.index(True)
            for action, keywords in self.actionKeywords.items():
                if(any(x in keywords for x in words[index:])):
                    self.actionQueue.put(action)
                    # print(action)
                    break

    def getAction(self):
        try:
            action = self.actionQueue.get(block=False)
        except queue.Empty:
            action = None
        return action

    def hasNewSentence(self):
        return self.sentenceTriggered

    def getLastRecognizedText(self):
        self.sentenceTriggered = False
        return self.lastRecognizedText

    def startVosk(self):
        self.vosk_rec.start()
    
    def stopVosk(self):
        self.vosk_rec.stop()

    def setDevice(self, device):
        self.vosk_rec.device = device

class BarkHandler:
    def __init__(self, device = -1):
        # === NEW: Load bark audio once at startup ===
        self.bark_data, self.bark_fs = sf.read("bark.wav", dtype='float32')
        if self.bark_data.ndim == 1:
            self.bark_data = np.column_stack([self.bark_data, self.bark_data])  # ensure stereo
        volume_factor = 100.0  # 1.0 = original, 2.0 = 2x louder
        self.bark_data = self.bark_data * volume_factor

        # Avoid clipping
        self.bark_data = np.clip(self.bark_data, -1.0, 1.0)
        self.device = device  # change to your USB speaker index

    # bark_data, bark_fs already loaded
    def play_bark(self):
        # Use a thread to avoid blocking the main loop
        sd.stop()  # stop previous playback
        threading.Thread(target=lambda: sd.play(self.bark_data, self.bark_fs, device=self.device)).start()



def main_curses(stdscr):
    # Clear screen and make getch non-blocking
    stdscr.clear()
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "Program running. Press 'q' to quit.")

    running = True

    # Initialize your handlers
    bh = BarkHandler(device=8)
    model_path = "./vosk-model-small-en-us-0.15"
    vosk_rec = VoskKeywordProcessor(model_path)
    
    vosk_rec.startVosk()

    while running:
        # Check for keypress
        key = stdscr.getch()
        if key != -1:
            if chr(key) == 'q':
                running = False
                stdscr.addstr(2, 0, "Quitting...")
                stdscr.refresh()
                break

        # Process actions
        action = vosk_rec.getAction()
        while action is not None:
            stdscr.addstr(3, 0, f"Action: {action}")
            stdscr.clrtoeol()
            stdscr.refresh()
            if action == VoskKeywordProcessor.Actions.BARK:
                bh.play_bark()
            action = vosk_rec.getAction()

        time.sleep(0.1)  # small delay for CPU

    vosk_rec.stopVosk()

if __name__ == "__main__":
    curses.wrapper(main_curses)
