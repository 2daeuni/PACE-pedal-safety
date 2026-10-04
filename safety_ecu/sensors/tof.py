import statistics
from collections import deque

import board
import adafruit_vl53l1x


class ToFSensor:
    def __init__(self, filter_size=5):
        self.filter_size = filter_size

        # I2C bus
        self.i2c = board.I2C()

        # VL53L1X sensor
        self.sensor = adafruit_vl53l1x.VL53L1X(
            self.i2c
        )

        # Median filter buffer
        self.distance_buffer = deque(
            maxlen=filter_size
        )

    def start(self):
        self.sensor.start_ranging()
        print("[ToF] Ranging started")

    def read_distance(self):
        try:
            # Check whether new measurement data is ready
            if not self.sensor.data_ready:
                return None, True

            # Read distance
            distance_cm = self.sensor.distance

            # Clear sensor interrupt
            self.sensor.clear_interrupt()

            # Communication succeeded,
            # but no valid distance is available
            if distance_cm is None:
                return None, True

            # Add measurement to median filter
            self.distance_buffer.append(
                distance_cm
            )

            # Calculate median
            filtered_distance = statistics.median(
                self.distance_buffer
            )

            return filtered_distance, True

        except (OSError, RuntimeError, ValueError):
            return None, False

    def stop(self):
        try:
            self.sensor.stop_ranging()
            print("[ToF] Ranging stopped")

        except (OSError, RuntimeError, ValueError):
            print("[ToF] Stop failed")

    def clear_filter(self):
        self.distance_buffer.clear()
