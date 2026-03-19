import time
import math
from statistics import pstdev
from typing import Optional, Dict, Any, cast
import pandas as pd
from collections import deque
import threading


class Spirometer:
    """
    Pure data processing API for ESP32 Spirometer (no BLE connection).
    Designed for use with Flutter apps that handle BLE communication.

    Core features:
      - Sample processing and storage
      - Calibration using earth field reference
      - Stability detection for motion analysis
      - Drift compensation during idle periods
      - Data export (CSV or pandas DataFrame)

    Data flow:
      - Sensor readings are provided by external code via add_sample()
      - Raw values are combined: dx = x1 - x2, dy = y1 + y2, dz = z1 + z2
      - Offsets are applied for calibration: cal_x = dx - offset_x, etc.
      - Stability is tracked and used for auto-calibration

    Usage:
      - Create instance: api = Spirometer()
      - Feed samples: api.add_sample(x1, y1, z1, x2, y2, z2)
      - Get latest data: api.get_latest_calibrated()
      - Calibrate: api.calibrate()
      - Export: api.export_calibrated_csv(filepath)
    """

    def __init__(
        self,
        auto_calibrate_on_first_samples: bool = False,
        calibration_samples: int = 200,
        # Stability and drift settings
        stability_window_seconds: float = 10.0,
        stability_min_samples: int = 30,
        stability_std_threshold: float = 10.0,
        idle_mag_threshold: float = 20.0,
        idle_required_seconds: float = 2.0,
        drift_alpha: float = 0.002,
        drift_enabled: bool = True,
    ) -> None:
        self.auto_calibrate_on_first_samples = auto_calibrate_on_first_samples
        self.calibration_samples = calibration_samples

        # Stability/drift config
        self.stability_window_seconds = float(stability_window_seconds)
        self.stability_min_samples = int(stability_min_samples)
        self.stability_std_threshold = float(stability_std_threshold)
        self.idle_mag_threshold = float(idle_mag_threshold)
        self.idle_required_seconds = float(idle_required_seconds)
        self.drift_alpha = float(drift_alpha)
        self.drift_enabled = bool(drift_enabled)

        # Offsets determined by calibration (for dx, dy, dz respectively)
        self._offset = [0.0, 0.0, 0.0]

        # Storage for incoming samples (thread-safe)
        self._samples: deque[Dict] = deque(maxlen=10000)
        self._lock = threading.Lock()

        # Calibration coordination
        self._calibrating = False
        self._calib_count = 0
        self._calib_sum = [0.0, 0.0, 0.0]

        # Stability/drift state
        self._stability_buf: deque[tuple[float, float, float, float]] = deque()
        self._stable: bool = False
        self._stability_stats: Dict[str, Any] = {"stdx": None, "stdy": None, "stdz": None, "count": 0}
        self._last_idle_start: Optional[float] = None

        # Track first samples for auto-calibration
        self._first_samples_received = 0

    # ------------------------ Stability helpers ------------------------
    def _update_stability(self, dx: float, dy: float, dz: float, t: float) -> None:
        """Maintain a time-window buffer and compute motion stability.
        Stable = low stddev in dx, dy, dz over recent window.
        """
        self._stability_buf.append((t, dx, dy, dz))
        while self._stability_buf and (t - self._stability_buf[0][0]) > self.stability_window_seconds:
            self._stability_buf.popleft()
        count = len(self._stability_buf)
        self._stability_stats["count"] = count
        if count >= self.stability_min_samples:
            xs = [v[1] for v in self._stability_buf]
            ys = [v[2] for v in self._stability_buf]
            zs = [v[3] for v in self._stability_buf]
            stdx = pstdev(xs)
            stdy = pstdev(ys)
            stdz = pstdev(zs)
            self._stability_stats.update({"stdx": stdx, "stdy": stdy, "stdz": stdz})
            self._stable = (stdx < self.stability_std_threshold and
                            stdy < self.stability_std_threshold and
                            stdz < self.stability_std_threshold)
        else:
            self._stable = False

    def is_stable(self) -> bool:
        """Return True if device motion appears stable in recent window."""
        return bool(self._stable)

    # ------------------------ Public API (sync) ------------------------
    def add_sample(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> None:
        """
        Add a sensor sample to the spirometer.
        
        Args:
            x1, y1, z1: Cilia field sensor readings
            x2, y2, z2: Earth field (reference) sensor readings
        """
        now_t = time.time()
        # Compose difference (cilia - earth)
        dx = x1 - x2
        dy = y1 + y2
        dz = z1 + z2

        self._update_stability(dx, dy, dz, now_t)

        # If calibrating, only accept stable samples
        if self._calibrating and self._stable:
            self._calib_sum[0] += dx
            self._calib_sum[1] += dy
            self._calib_sum[2] += dz
            self._calib_count += 1

        # Apply offset
        cal_x = dx - self._offset[0]
        cal_y = dy - self._offset[1]
        cal_z = dz - self._offset[2]

        # Idle detection based on magnitude of calibrated vector
        mag = math.sqrt(cal_x * cal_x + cal_y * cal_y + cal_z * cal_z)
        if mag < self.idle_mag_threshold and self._stable:
            if self._last_idle_start is None:
                self._last_idle_start = now_t
            elif self.drift_enabled and (now_t - self._last_idle_start) >= self.idle_required_seconds:
                self._offset[0] = (1.0 - self.drift_alpha) * self._offset[0] + self.drift_alpha * dx
                self._offset[1] = (1.0 - self.drift_alpha) * self._offset[1] + self.drift_alpha * dy
                self._offset[2] = (1.0 - self.drift_alpha) * self._offset[2] + self.drift_alpha * dz
        else:
            self._last_idle_start = None

        row = {
            "t": now_t,
            "x1": x1, "y1": y1, "z1": z1,
            "x2": x2, "y2": y2, "z2": z2,
            "dx": dx, "dy": dy, "dz": dz,
            "cal_x": cal_x, "cal_y": cal_y, "cal_z": cal_z,
        }
        with self._lock:
            self._samples.append(row)
            self._first_samples_received += 1
            # Auto-calibrate on first N samples if enabled
            if (self.auto_calibrate_on_first_samples and 
                self._first_samples_received == self.calibration_samples and
                not self._calibrating):
                self.calibrate(samples=self.calibration_samples)

    def calibrate(self, samples: Optional[int] = None, timeout: float = 10.0) -> None:
        """
        Calibrate offsets using current streaming data.
        Collect N stable samples of (x1-x2, y1+y2, z1+z2) and set offset to their mean.
        
        Args:
            samples: Number of stable samples to collect (default: calibration_samples)
            timeout: Maximum time to wait for calibration (seconds)
        """
        n = int(samples if samples is not None else self.calibration_samples)
        self._calibrating = True
        self._calib_count = 0
        self._calib_sum = [0.0, 0.0, 0.0]

        # Wait until enough stable samples pass through add_sample
        start_t = time.time()
        while self._calib_count < n:
            time.sleep(0.01)
            if time.time() - start_t > timeout:
                break

        # Finalize
        if self._calib_count <= 0:
            self._calibrating = False
            raise RuntimeError("Calibration failed: no stable samples collected.")
        self._offset = [s / self._calib_count for s in self._calib_sum]
        self._calibrating = False

    def export_calibrated_csv(self, filepath: str) -> str:
        """Export stored samples with calibrated columns to CSV."""
        with self._lock:
            rows = list(self._samples)
        if not rows:
            raise RuntimeError("No samples to export.")
        df = pd.DataFrame(rows)
        ordered_cols = [
            "t",
            "x1", "y1", "z1",
            "x2", "y2", "z2",
            "dx", "dy", "dz",
            "cal_x", "cal_y", "cal_z",
        ]
        cols = [c for c in ordered_cols if c in df.columns] + [c for c in df.columns if c not in ordered_cols]
        df = df[cols]
        df.to_csv(filepath, index=False)
        return filepath

    def export_calibrated_dataframe(self) -> pd.DataFrame:
        """Return a pandas DataFrame of stored samples with calibrated columns."""
        with self._lock:
            rows = list(self._samples)
        if not rows:
            raise RuntimeError("No samples to export.")
        df = pd.DataFrame(rows)
        ordered_cols = [
            "t",
            "x1", "y1", "z1",
            "x2", "y2", "z2",
            "dx", "dy", "dz",
            "cal_x", "cal_y", "cal_z",
        ]
        cols = [c for c in ordered_cols if c in df.columns] + [c for c in df.columns if c not in ordered_cols]
        return df[cols]

    # ------------------------ Realtime accessors (sync) ------------------------
    def get_latest_sample(self) -> Optional[Dict]:
        """Return the most recent sample dict or None if empty."""
        with self._lock:
            if not self._samples:
                return None
            return self._samples[-1]

    def get_latest_calibrated(self) -> Optional[tuple[float, float, float]]:
        """Return latest (cal_x, cal_y, cal_z) if available."""
        s = self.get_latest_sample()
        if not s:
            return None
        raw_cx = s.get("cal_x")
        raw_cy = s.get("cal_y")
        raw_cz = s.get("cal_z")

        cx_val = cast(float, raw_cx) if raw_cx is not None else (float(s.get("dx", 0.0)) - float(self._offset[0]))
        cy_val = cast(float, raw_cy) if raw_cy is not None else (float(s.get("dy", 0.0)) - float(self._offset[1]))
        cz_val = cast(float, raw_cz) if raw_cz is not None else (float(s.get("dz", 0.0)) - float(self._offset[2]))

        return float(cx_val), float(cy_val), float(cz_val)

    def get_latest_calibrated_x(self, default: float = 0.0) -> float:
        """Return latest calibrated X value or default if none available."""
        vals = self.get_latest_calibrated()
        if not vals:
            return default
        return float(vals[0])

    def is_idle(
        self,
        idle_x_threshold: float = 12.0,
        idle_ratio_required: float = 0.85,
        window_seconds: float = 20.0
    ) -> bool:
        """
        Return True if the device has been idle:
        - For at least idle_ratio_required of the last window_seconds,
        - |cal_x - avg_x| < idle_x_threshold
        """
        with self._lock:
            samples = [s for s in self._samples if "cal_x" in s]
        if not samples:
            return False
        now = time.time()
        window_samples = [s for s in samples if now - s["t"] <= window_seconds]
        if len(window_samples) < 5:
            return False
        avg_x = sum(s["cal_x"] for s in window_samples) / len(window_samples)
        idle_samples = [abs(s["cal_x"] - avg_x) < idle_x_threshold for s in window_samples]
        idle_ratio = sum(idle_samples) / len(idle_samples)
        return idle_ratio > idle_ratio_required

    def get_idle_avg_x(self, window_seconds: float = 20.0) -> Optional[float]:
        """
        Return the average cal_x over the last window_seconds, or None if not enough samples.
        """
        with self._lock:
            samples = [s for s in self._samples if "cal_x" in s]
        if not samples:
            return None
        now = time.time()
        window_samples = [s for s in samples if now - s["t"] <= window_seconds]
        if len(window_samples) < 5:
            return None
        avg_val = sum(s["cal_x"] for s in window_samples) / len(window_samples)
        return avg_val

    def get_calibration_offset(self) -> tuple[float, float, float]:
        """Return current calibration offsets (offset_x, offset_y, offset_z)."""
        return (self._offset[0], self._offset[1], self._offset[2])

    def set_calibration_offset(self, offset_x: float, offset_y: float, offset_z: float) -> None:
        """Manually set calibration offsets."""
        self._offset = [float(offset_x), float(offset_y), float(offset_z)]

    def clear_samples(self) -> None:
        """Clear all stored samples."""
        with self._lock:
            self._samples.clear()

    def get_sample_count(self) -> int:
        """Return the number of stored samples."""
        with self._lock:
            return len(self._samples)

    def get_stability_stats(self) -> Dict[str, Any]:
        """Return current stability statistics."""
        return dict(self._stability_stats)
