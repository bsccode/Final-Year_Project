#/frontend/system_stats.py

import psutil
import time
import threading

class SystemStats:
    def __init__(self):
        self.stop_flag = False
        self.stats = {}

    def start(self):
        """Start collecting stats in a separate thread."""
        self.stop_flag = False
        threading.Thread(target=self._collect_stats, daemon=True).start()

    def stop(self):
        """Stop the stats collection."""
        self.stop_flag = True

    def _collect_stats(self):
        """Collect system stats periodically."""
        while not self.stop_flag:
            self.stats = {
                "cpu_percent": psutil.cpu_percent(percpu=True),
                "memory_percent": psutil.virtual_memory().percent,
                "disk_usage_percent": psutil.disk_usage('/').percent,
                "net_io": psutil.net_io_counters(),
                "temperature": self._get_temperature(),
            }
            time.sleep(0.5)  # Collect every 3 seconds

    def _get_temperature(self):
        """Get CPU temperature if available."""
        try:
            temperatures = psutil.sensors_temperatures()
            if "coretemp" in temperatures:
                # Return the average temperature
                return sum([t.current for t in temperatures["coretemp"]]) / len(temperatures["coretemp"])
            else:
                return None
        except (AttributeError, KeyError):
            return None

    def get_stats(self):
        """Return the latest stats."""
        return self.stats
