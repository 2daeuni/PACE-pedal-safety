class PMSCalculator:
    def __init__(self):
        self.score = 0.0
        self.available = False

    def calculate(
        self,
        accelerator,
        accelerator_rate,
        pedal_connected,
        sensor_status,
    ):
        """
        Calculate PMS risk.

        PMS inputs:
        - accelerator pedal input
        - accelerator pedal input rate

        Sensor Status:
        - 0x00 = normal
        - non-zero = one or more sensor errors

        Actual PMS risk thresholds are not
        implemented yet.
        """

        # Pedal ECU communication failure
        if not pedal_connected:
            self.score = 0.0
            self.available = False

            return self.score

        # Pedal sensor error
        if sensor_status != 0x00:
            self.score = 0.0
            self.available = False

            return self.score

        # Accelerator data unavailable
        if accelerator is None:
            self.score = 0.0
            self.available = False

            return self.score

        # Accelerator rate unavailable
        if accelerator_rate is None:
            self.score = 0.0
            self.available = False

            return self.score

        # -------------------------------------------------
        # PMS risk algorithm will be implemented later.
        # Thresholds will be tuned using actual pedal data.
        # -------------------------------------------------

        self.score = 0.0
        self.available = True

        return self.score