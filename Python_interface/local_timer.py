# timer.py
# coding: utf-8
DEBUG = False

from observer import Subject, Observer
import threading
import time


class Timer(Subject):
    def __init__(self, period):
        super().__init__()
        # Validate period
        if period is None:
            raise ValueError("period must be provided")
        try:
            self.period = float(period)
        except Exception:
            raise ValueError("period must be a number")

        if self.period <= 0:
            raise ValueError("period must be > 0")

        self._stop_event = threading.Event()
        self._thread = None
    
    def start(self):
        """Start the timer. If already running, this is a no-op."""
        if self._thread is not None and self._thread.is_alive():
            if DEBUG:
                print(f"{type(self).__name__} - already running")
            return

        # Clear any previous stop request
        self._stop_event.clear()

        def _run_loop():
            if DEBUG:
                print(f"{type(self).__name__} - thread started, period=", self.period)
            # Loop until stop event is set. Wait returns True if event set.
            while not self._stop_event.is_set():
                waited = self._stop_event.wait(self.period)
                if waited:
                    # stop requested
                    break
                try:
                    if DEBUG:
                        print(f"{type(self).__name__} - notifying observers")
                    self.notify()
                except Exception as e:
                    # Don't let observer exceptions kill the loop; log and continue
                    if DEBUG:
                        print(f"{type(self).__name__} - exception while notifying observers:", e)
            if DEBUG:
                print(f"{type(self).__name__} - thread exiting")

        self._thread = threading.Thread(target=_run_loop, daemon=True)
        self._thread.start()
    def stop(self):
        """Stop the timer. If not running, this is a no-op."""
        if self._thread is None:
            if DEBUG:
                print(f"{type(self).__name__} - not running")
            return

        # Signal the thread to stop and wait for it to exit
        self._stop_event.set()
        if self._thread.is_alive():
            # Wait a short while for clean exit
            self._thread.join(timeout=self.period + 1.0)
        if DEBUG:
            print(f"{type(self).__name__} - stopped")
        # Clear thread reference
        self._thread = None

class PrintObserver(Observer):
    def __init__(self):
        self.count = 0
    def update(self, subject):
        self.count += 1
        print(f"{type(self).__name__} - {self.count}")

if __name__ == '__main__':
    t = Timer(0.5)
    p = PrintObserver()
    t.attach(p)
    t.start()
    # Let it run for a bit (expect ~3 updates at 0.5s intervals over 1.6s)
    time.sleep(1.6)
    t.stop()
    print(f"final count: {p.count}")

    