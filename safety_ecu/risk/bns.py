class BNSCalculator:
    def __init__(self):
        self.score = 0.0
        self.available = False

    def calculate(
        self,
        distance_cm,
        distance_valid,
        vehicle_speed,
        ttc,
    ):
        """
        Calculate BNS risk.

        BNS inputs:
        - obstacle distance
        - vehicle speed
        - TTC

        Risk calculation is not implemented yet
        because vehicle speed and TTC are not
        currently available.
        """

        # Distance data is not usable
        if not distance_valid:
            self.score = 0.0
            self.available = False

            return self.score

        # Vehicle speed is not available
        if vehicle_speed is None:
            self.score = 0.0
            self.available = False

            return self.score

        # TTC is not available
        if ttc is None:
            self.score = 0.0
            self.available = False

            return self.score

        # -------------------------------------------------
        # BNS algorithm will be implemented here later.
        # -------------------------------------------------

        self.score = 0.0
        self.available = True

        return self.score
