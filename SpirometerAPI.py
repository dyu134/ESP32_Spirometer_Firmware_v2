import asyncio
import threading
import time
import math
from statistics import pstdev
from typing import Optional, List, Dict, Any, cast
from bleak import BleakClient, BleakScanner
import pandas as pd
from collections import deque


class Spirometer:
    """
    Python BLE client for ESP32 Spirometer.
    - Streams raw values: (x1, y1, z1) cilia field with offset; (x2, y2, z2) earth field
    - Calibrates cilia field using earth field: cal = (x1 - x2, y1 - y2, z1 - z2) - offset
    - Exports calibrated data (CSV)
    - Reads current battery level (%)
    """

    def __init__(
        self,
        device_id: Optional[str] = "B4:3A:45:34:A2:95",
        data_service_uuid: str = "4fafc201-1fb5-459e-8fcc-c5c9c331914b",
        data_char_uuid: str = "beb5483e-36e1-4688-b7f5-ea07361b26a8",
        battery_service_uuid: str = "0000180F-0000-1000-8000-00805F9B34FB",
        battery_char_uuid: str = "00002A19-0000-1000-8000-00805F9B34FB",
        auto_calibrate_on_connect: bool = True,
        calibration_samples: int = 200,
        # Stability and drift settings
        stability_window_seconds: float = 1.0,
        stability_min_samples: int = 30,
        stability_std_threshold: float = 5.0,
        idle_mag_threshold: float = 20.0,
        idle_required_seconds: float = 2.0,
        drift_alpha: float = 0.002,
        drift_enabled: bool = True,
    ) -> None:
        self.device_id = device_id
        self.data_service_uuid = data_service_uuid
        self.data_char_uuid = data_char_uuid
        self.battery_service_uuid = battery_service_uuid
        self.battery_char_uuid = battery_char_uuid
        self.auto_calibrate_on_connect = auto_calibrate_on_connect
        self.calibration_samples = calibration_samples

        # Stability/drift config
        self.stability_window_seconds = float(stability_window_seconds)
        self.stability_min_samples = int(stability_min_samples)
        self.stability_std_threshold = float(stability_std_threshold)
        self.idle_mag_threshold = float(idle_mag_threshold)
        self.idle_required_seconds = float(idle_required_seconds)
        self.drift_alpha = float(drift_alpha)
        self.drift_enabled = bool(drift_enabled)

        self._client: Optional[BleakClient] = None

        # Background asyncio loop
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

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

    # ------------------------ Async loop plumbing ------------------------
    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_coro(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result()

    # ------------------------ Stability helpers ------------------------
    def _update_stability(self, dx: float, dy: float, dz: float, t: float) -> None:
        """Maintain a time-window buffer and compute motion stability.
        Stable = low stddev in dx, dy, dz over recent window.
        """
        self._stability_buf.append((t, dx, dy, dz))
        # Drop old samples outside time window
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
    def discover_and_set_device(self, timeout: float = 8.0) -> Optional[str]:
        """Scan for device advertising the data service and set device_id."""
        async def _scan():
            # Use advertisement data (type-safe) to find service UUIDs
            def _filter(device, adv_data):
                svc_uuids = (getattr(adv_data, "service_uuids", None) or [])
                return any(u.lower() == self.data_service_uuid.lower() for u in svc_uuids)

            dev = await BleakScanner.find_device_by_filter(_filter, timeout=timeout)
            return dev.address if dev else None

        addr = self._run_coro(_scan())
        if addr:
            self.device_id = addr
        return addr

    def connect(self) -> None:
        """Connect and start streaming. Auto-calibrate if enabled."""
        if not self.device_id:
            raise RuntimeError("device_id not set. Call discover_and_set_device() or provide device_id.")
        self._run_coro(self._async_connect())

    def disconnect(self) -> None:
        self._run_coro(self._async_disconnect())

    def calibrate(self, samples: Optional[int] = None, timeout: float = 10.0) -> None:
        """Calibrate offsets using current streaming data.
        Collect N samples of (x1-x2, y1-y2, z1-z2) and set offset to their mean.
        """
        self._run_coro(self._async_calibrate(samples=samples, timeout=timeout))

    def export_calibrated_csv(self, filepath: str) -> str:
        """Export stored samples with calibrated columns to CSV."""
        with self._lock:
            rows = list(self._samples)
        if not rows:
            raise RuntimeError("No samples to export.")
        df = pd.DataFrame(rows)
        # Ensure ordered columns if present
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
        # Fallback to compute from dx/dy/dz if cal_* not present
        # Use local variables to satisfy type checkers (avoid Optional/Unknown)
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

    def get_battery_level(self) -> int:
        """Read current battery level (%) once from the Battery Level characteristic."""
        return self._run_coro(self._async_read_battery())

    # ------------------------ BLE internals (async) ------------------------
    async def _async_connect(self) -> None:
        if self._client and self._client.is_connected:
            return
        # Ensure device_id is set and typed as str for the BLE client
        dev_id = self.device_id
        if dev_id is None:
            raise RuntimeError("device_id not set. Call discover_and_set_device() or provide device_id.")
        self._client = BleakClient(dev_id)
        await self._client.connect()
        # Start streaming notifications
        await self._client.start_notify(self.data_char_uuid, self._on_data)
        # Optionally auto-calibrate
        if self.auto_calibrate_on_connect:
            await self._async_calibrate(samples=self.calibration_samples)

    async def _async_disconnect(self) -> None:
        if self._client:
            try:
                await self._client.stop_notify(self.data_char_uuid)
            except Exception:
                pass
            if self._client.is_connected:
                await self._client.disconnect()
            self._client = None

    async def _async_read_battery(self) -> int:
        if not self._client or not self._client.is_connected:
            raise RuntimeError("Not connected.")
        data = await self._client.read_gatt_char(self.battery_char_uuid)
        if not data:
            return 0
        return int(data[0])

    async def _async_calibrate(self, samples: Optional[int] = None, timeout: float = 10.0) -> None:
        """Calibrate offsets using stable samples only.
        Rejects samples when motion is unstable (shake/rotation).
        """
        if not self._client or not self._client.is_connected:
            raise RuntimeError("Not connected.")
        n = int(samples if samples is not None else self.calibration_samples)
        # Prepare accumulators
        self._calibrating = True
        self._calib_count = 0
        self._calib_sum = [0.0, 0.0, 0.0]

        # Wait until enough stable samples pass through notification handler
        start_t = time.time()
        while self._calib_count < n:
            await asyncio.sleep(0.01)
            if time.time() - start_t > timeout:
                break
        # Finalize
        if self._calib_count <= 0:
            self._calibrating = False
            raise RuntimeError("Calibration failed: device not stable long enough.")
        self._offset = [s / self._calib_count for s in self._calib_sum]
        self._calibrating = False

    # ------------------------ Notification parser ------------------------
    def _on_data(self, _char: Any, data: bytearray) -> None:
        try:
            txt = data.decode(errors="ignore").strip()
            parts = txt.split()
            # Expected: s <x1> x1 <z1> z1 <x2> x2 <z2> z2 <y1> y1 <y2> y2
            # Extract floats at indices 1,3,5,7,9,11 if present
            vals: List[float] = []
            for i in (1, 3, 5, 7, 9, 11):
                if i < len(parts):
                    try:
                        vals.append(float(parts[i]))
                    except ValueError:
                        vals.append(float('nan'))
                else:
                    vals.append(float('nan'))
            x1, z1, x2, z2, y1, y2 = vals
            # Compose difference (cilia - earth) (sensor 2 is placed upside-down with x axis aligned with sensor 1)
            dx = x1 - x2
            dy = y1 + y2
            dz = z1 + z2

            now_t = time.time()
            # Update stability with raw diffs
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
                # Start or continue idle window
                if self._last_idle_start is None:
                    self._last_idle_start = now_t
                # Apply slow drift correction if idle long enough
                elif self.drift_enabled and (now_t - self._last_idle_start) >= self.idle_required_seconds:
                    # Exponential moving average towards current raw diffs
                    self._offset[0] = (1.0 - self.drift_alpha) * self._offset[0] + self.drift_alpha * dx
                    self._offset[1] = (1.0 - self.drift_alpha) * self._offset[1] + self.drift_alpha * dy
                    self._offset[2] = (1.0 - self.drift_alpha) * self._offset[2] + self.drift_alpha * dz
            else:
                # Reset idle timer on activity or instability
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
        except Exception:
            # Ignore malformed packets
            return


# ------------------------ Simple usage example (commented) ------------------------
if __name__ == "__main__":
    api = Spirometer(device_id="9C:13:9E:9D:20:C1", auto_calibrate_on_connect=True, calibration_samples=300)
    api.connect()
    time.sleep(5)  # collect some data
    print("Battery:", api.get_battery_level(), "%")
    api.export_calibrated_csv("spirometer_data.csv")
    api.disconnect()