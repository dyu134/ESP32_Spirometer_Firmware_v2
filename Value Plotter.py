import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time
from SpirometerAPI import Spirometer


class RealtimePlotter:
    def __init__(self, device_id: str = "B4:3A:45:34:A2:95", max_points: int = 500):
        self.spirometer = Spirometer(
            device_id=device_id,
            auto_calibrate_on_connect=True,
            calibration_samples=100
        )
        self.max_points = max_points
        self.raw_x_values = deque(maxlen=max_points)
        self.cal_x_values = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.start_time = None
        self.connected = False
        self.connection_error = None

    def connect_background(self):
        def worker():
            try:
                print("Connecting to spirometer...")
                self.spirometer.connect()
                self.connected = True
                print("Connected successfully")
            except Exception as e:
                self.connected = False
                self.connection_error = str(e)
                print(f"Connection failed: {e}")
        
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def update_data(self):
        if not self.connected:
            return
        
        sample = self.spirometer.get_latest_sample()
        if sample:
            if self.start_time is None:
                self.start_time = sample.get("t", time.time())
            
            elapsed = sample.get("t", time.time()) - self.start_time
            raw_x = sample.get("dx", 0.0)
            cal_x = sample.get("cal_x", 0.0)
            
            self.timestamps.append(elapsed)
            self.raw_x_values.append(raw_x)
            self.cal_x_values.append(cal_x)

    def setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("X Value (μT)")
        self.ax.set_title("Spirometer Real-time X-axis Plot")
        self.ax.grid(True, alpha=0.3)
        
        self.line_raw, = self.ax.plot([], [], label="Raw X (dx)", color="steelblue", linewidth=2)
        self.line_cal, = self.ax.plot([], [], label="Calibrated X (cal_x)", color="coral", linewidth=2)
        self.ax.legend(loc="upper left")

    def animate(self, frame):
        self.update_data()
        
        if not self.connected and frame > 0 and not self.timestamps:
            status = self.connection_error or "Connecting..."
            self.ax.set_title(f"Spirometer Real-time X-axis Plot - {status}")
            return self.line_raw, self.line_cal
        
        if self.timestamps:
            times = list(self.timestamps)
            self.line_raw.set_data(times, list(self.raw_x_values))
            self.line_cal.set_data(times, list(self.cal_x_values))
            
            if times:
                margin = (times[-1] - times[0]) * 0.1 if times[-1] > times[0] else 1
                self.ax.set_xlim(max(0, times[-1] - 50), times[-1] + 2)
                
                all_values = list(self.raw_x_values) + list(self.cal_x_values)
                if all_values:
                    y_min, y_max = min(all_values), max(all_values)
                    y_margin = (y_max - y_min) * 0.1 if y_max > y_min else 10
                    self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
        
        status = "Connected" if self.connected else (self.connection_error or "Connecting...")
        self.ax.set_title(f"Spirometer Real-time X-axis Plot - {status}")
        
        return self.line_raw, self.line_cal

    def run(self):
        self.connect_background()
        time.sleep(1)
        
        self.setup_plot()
        anim = FuncAnimation(self.fig, self.animate, interval=50, blit=True, cache_frame_data=False)
        
        plt.show()
        
        try:
            self.spirometer.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    plotter = RealtimePlotter()
    plotter.run()
