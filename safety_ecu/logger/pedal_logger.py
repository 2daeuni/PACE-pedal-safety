import csv
import time


class PedalLogger:
    def __init__(self, filename="pedal_data.csv"):
        self.filename = filename

        with open(self.filename, "w", newline="") as file:
            writer = csv.writer(file)

            writer.writerow([
                "time",
                "accelerator",
                "brake",
                "accelerator_rate",
                "brake_rate",
            ])

    def log(self, state):
        with open(self.filename, "a", newline="") as file:
            writer = csv.writer(file)

            writer.writerow([
                time.monotonic(),
                state.accelerator,
                state.brake,
                state.accelerator_rate,
                state.brake_rate,
            ])
