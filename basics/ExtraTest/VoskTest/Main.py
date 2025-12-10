# main_pi.py
import time
import curses
from VoskTest import VoskKeywordProcessor, BarkHandler  # assuming your classes are in VoskTest.py

def main_curses(stdscr):
    # Clear screen and non-blocking input
    stdscr.clear()
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "Vosk Demo on Pi. Press 'q' to quit.")
    stdscr.refresh()

    running = True

    # Initialize BarkHandler and Vosk
    bh = BarkHandler(device=1)  # default output device
    model_path = "./vosk-model-small-en-us-0.15"
    vosk_rec = VoskKeywordProcessor(model_path, device=2)

    # Optionally list audio devices
    stdscr.addstr(2, 0, "Available audio devices:")
    stdscr.refresh()
    devices = vosk_rec.vosk_rec.list_devices()

    # Start Vosk recognition
    vosk_rec.startVosk()
    stdscr.addstr(4, 0, "Recognition started...")
    stdscr.refresh()

    try:
        while running:
            # Non-blocking key detection
            key = stdscr.getch()
            if key != -1:
                if chr(key) == 'q':
                    running = False
                    stdscr.addstr(6, 0, "Quitting...")
                    stdscr.refresh()
                    break

            # Process queued actions
            action = vosk_rec.getAction()
            while action is not None:
                stdscr.addstr(5, 0, f"Detected action: {action}      ")
                stdscr.refresh()
                if action == VoskKeywordProcessor.Actions.BARK:
                    bh.play_bark()
                action = vosk_rec.getAction()

            time.sleep(0.1)

    except KeyboardInterrupt:
        stdscr.addstr(6, 0, "Interrupted by user.")
        stdscr.refresh()

    finally:
        vosk_rec.stopVosk()
        stdscr.addstr(7, 0, "Program exited cleanly.")
        stdscr.refresh()
        time.sleep(1)

if __name__ == "__main__":
    import curses
    curses.wrapper(main_curses)
