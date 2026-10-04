import time


class SystemState:
    def __init__(self):
        # -------------------------------------------------
        # Pedal
        # -------------------------------------------------
        self.accelerator = 0
        self.brake = 0

        self.accelerator_rate = 0.0
        self.brake_rate = 0.0

        self.pedal_sensor_status = None
        self.pedal_alive_counter = None

        self.pedal_connected = False

        # -------------------------------------------------
        # Environment / ToF
        # -------------------------------------------------
        self.distance_cm = None

        # ToF communication state
        self.tof_connected = False

        # Distance data validity
        self.distance_valid = False

        # Time when the latest valid distance
        # measurement was received
        self.last_distance_time = None

        # -------------------------------------------------
        # Vehicle
        # -------------------------------------------------
        self.vehicle_speed = None
        self.ttc = None

        # -------------------------------------------------
        # Risk
        # -------------------------------------------------
        self.bns = 0.0
        self.pms = 0.0

        self.total_risk = 0.0
        self.risk_level = 0

    def update_pedal(self, pedal):
        self.accelerator = pedal["accelerator"]
        self.brake = pedal["brake"]

        self.accelerator_rate = (
            pedal["accelerator_rate"]
        )

        self.brake_rate = (
            pedal["brake_rate"]
        )

        self.pedal_sensor_status = (
            pedal["sensor_status"]
        )

        self.pedal_alive_counter = (
            pedal["alive_counter"]
        )

        self.pedal_connected = True

    def update_distance(self, distance_cm):
        self.distance_cm = distance_cm

        self.tof_connected = True
        self.distance_valid = True

        self.last_distance_time = (
            time.monotonic()
        )

    def invalidate_distance(self):
        """
        Mark the current distance value as invalid.

        The last measured distance is kept for
        diagnostics, but must not be used for
        risk calculation.
        """

        self.distance_valid = False

    def check_distance_freshness(
        self,
        timeout=0.5,
    ):
        """
        Invalidate the distance if a new valid
        measurement has not been received within
        the specified timeout.

        This does not change tof_connected.
        """

        if not self.distance_valid:
            return False

        if self.last_distance_time is None:
            self.distance_valid = False
            return False

        elapsed = (
            time.monotonic()
            - self.last_distance_time
        )

        if elapsed >= timeout:
            self.distance_valid = False
            return False

        return True