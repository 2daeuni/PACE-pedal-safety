import csv
import time


class SystemLogger:
    def __init__(self, filename="system_data.csv"):
        self.filename = filename

        with open(
            self.filename,
            "w",
            newline="",
        ) as file:
            writer = csv.writer(file)

            writer.writerow([
                "time",

                "accelerator",
                "brake",
                "accelerator_rate",
                "brake_rate",

                "pedal_connected",
                "pedal_sensor_status",

                "tof_connected",
                "distance_valid",
                "distance_cm",

                "vehicle_speed",
                "ttc",

                "bns",
                "bns_available",

                "pms",
                "pms_available",

                "total_risk",
                "risk_level",
                "risk_available",

                "failsafe_active",
                "failsafe_reason",

                "haptic_vibration_command",
                "haptic_intensity",
                "haptic_frequency",

                "motor_control_mode",
                "motor_output_limit",
                "motor_command_flags",
            ])

    def log(
        self,
        state,
        bns_available,
        pms_available,
        risk_available,
        failsafe_active,
        failsafe_reason,
        commands,
    ):
        with open(
            self.filename,
            "a",
            newline="",
        ) as file:
            writer = csv.writer(file)

            writer.writerow([
                time.monotonic(),

                state.accelerator,
                state.brake,
                state.accelerator_rate,
                state.brake_rate,

                state.pedal_connected,
                state.pedal_sensor_status,

                state.tof_connected,
                state.distance_valid,
                state.distance_cm,

                state.vehicle_speed,
                state.ttc,

                state.bns,
                bns_available,

                state.pms,
                pms_available,

                state.total_risk,
                state.risk_level,
                risk_available,

                failsafe_active,
                failsafe_reason,

                commands["haptic"][
                    "vibration_command"
                ],
                commands["haptic"][
                    "intensity"
                ],
                commands["haptic"][
                    "frequency"
                ],

                commands["motor"][
                    "control_mode"
                ],
                commands["motor"][
                    "output_limit"
                ],
                commands["motor"][
                    "command_flags"
                ],
            ])
