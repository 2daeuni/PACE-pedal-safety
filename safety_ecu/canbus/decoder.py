import struct


PEDAL_STATUS_ID = 0x100
HAPTIC_STATUS_ID = 0x200


SENSOR_STATUS_BITS = {
    0x01: "ACCEL_ADC_RANGE_ERROR",
    0x02: "BRAKE_ADC_RANGE_ERROR",
    0x04: "ACCEL_DISCONNECT",
    0x08: "BRAKE_DISCONNECT",
    0x10: "ACCEL_CALIBRATION_ERROR",
    0x20: "BRAKE_CALIBRATION_ERROR",
}


def decode_sensor_status(sensor_status):
    """
    Decode Sensor Status bitmask.

    0x00:
        Normal

    Multiple errors can be active at the same time.
    Example:
        0x09 =
        ACCEL_ADC_RANGE_ERROR
        + BRAKE_DISCONNECT
    """

    if sensor_status == 0x00:
        return ["NORMAL"]

    errors = []

    for bit, name in SENSOR_STATUS_BITS.items():
        if sensor_status & bit:
            errors.append(name)

    # Reserved bits
    if sensor_status & 0x40:
        errors.append("RESERVED_BIT_6")

    if sensor_status & 0x80:
        errors.append("RESERVED_BIT_7")

    return errors


def decode_pedal_status(data: bytes):
    if len(data) != 8:
        raise ValueError(
            f"PEDAL_STATUS must be 8 bytes, "
            f"got {len(data)}"
        )

    accelerator = data[0]
    brake = data[1]

    if not 0 <= accelerator <= 100:
        raise ValueError(
            f"Accelerator out of range: "
            f"{accelerator}%"
        )

    if not 0 <= brake <= 100:
        raise ValueError(
            f"Brake out of range: "
            f"{brake}%"
        )

    accelerator_rate = (
        struct.unpack_from(
            "<h",
            data,
            2,
        )[0]
        * 0.1
    )

    brake_rate = (
        struct.unpack_from(
            "<h",
            data,
            4,
        )[0]
        * 0.1
    )

    sensor_status = data[6]
    alive_counter = data[7]

    sensor_errors = decode_sensor_status(
        sensor_status
    )

    return {
        "accelerator": accelerator,
        "brake": brake,
        "accelerator_rate": accelerator_rate,
        "brake_rate": brake_rate,
        "sensor_status": sensor_status,
        "sensor_errors": sensor_errors,
        "sensor_normal": sensor_status == 0x00,
        "alive_counter": alive_counter,
    }

def decode_haptic_status(data: bytes):
    if len(data) != 8:
        raise ValueError(
            f"HAPTIC_STATUS must be 8 bytes, "
            f"got {len(data)}"
        )

    risk_level = data[0]
    vibration_status = data[1]
    intensity = data[2]
    frequency = data[3]
    fault_code = data[4]
    command_timeout = data[5]
    ecu_status = data[6]
    alive_counter = data[7]

    if not 0 <= risk_level <= 3:
        raise ValueError(
            f"Risk level out of range: "
            f"{risk_level}"
        )

    if vibration_status not in (0, 1):
        raise ValueError(
            f"Vibration status invalid: "
            f"{vibration_status}"
        )
    if vibration_status == 0:
        if intensity != 0 or frequency != 0:
            raise ValueError(
                "Haptic OFF state must have "
                "intensity=0 and frequency=0"
            )

    else:
        if not 1 <= intensity <= 100:
            raise ValueError(
                f"Haptic intensity out of range: "
                f"{intensity}%"
            )

        if not 10 <= frequency <= 50:
            raise ValueError(
                f"Haptic frequency out of range: "
                f"{frequency}Hz"
            )

    if command_timeout not in (0, 1):
        raise ValueError(
            f"Command timeout invalid: "
            f"{command_timeout}"
        )

    if ecu_status not in (0, 1):
        raise ValueError(
            f"ECU status invalid: "
            f"{ecu_status}"
        )

    return {
        "risk_level": risk_level,
        "vibration_status": vibration_status,
        "intensity": intensity,
        "frequency": frequency,
        "fault_code": fault_code,
        "command_timeout": command_timeout,
        "ecu_status": ecu_status,
        "alive_counter": alive_counter,
    }

if __name__ == "__main__":
    test_data = bytes.fromhex(
        "48 00 AC 0D 00 00 00 37"
    )

    result = decode_pedal_status(
        test_data
    )

    print(result)