class FailSafeManager:
    def __init__(self):
        self.active = False
        self.reason = None

    def evaluate(self, state):

        # ToF communication failure
        if not state.tof_connected:
            self.active = True
            self.reason = "TOF_COMMUNICATION_ERROR"

            return self.active

        # Pedal sensor error
        #
        # Only evaluate Sensor Status while the
        # Pedal ECU is currently connected.
        if (
            state.pedal_connected
            and state.pedal_sensor_status is not None
            and state.pedal_sensor_status != 0x00
        ):
            self.active = True
            self.reason = "PEDAL_SENSOR_ERROR"

            return self.active

        # No fault
        self.active = False
        self.reason = None

        return self.active
