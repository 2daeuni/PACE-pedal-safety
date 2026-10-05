import time
import multiprocessing
import queue

from canbus.receiver import CANReceiver
from canbus.sender import CANSender
from canbus.decoder import (
    PEDAL_STATUS_ID,
    HAPTIC_STATUS_ID,
    decode_pedal_status,
    decode_haptic_status,
)

from control.state import SystemState
from control.failsafe import FailSafeManager
from control.command_mapper import CommandMapper

from logger.pedal_logger import PedalLogger
from logger.system_logger import SystemLogger
from sensors.tof_worker import tof_worker

from risk.bns import BNSCalculator
from risk.pms import PMSCalculator
from risk.risk_manager import RiskManager


DISTANCE_FRESHNESS_TIMEOUT = 0.5


def start_tof_process(data_queue):
    """
    Start a new ToF worker process.
    """

    process = multiprocessing.Process(
        target=tof_worker,
        args=(data_queue,),
        daemon=True,
    )

    process.start()

    print(
        f"[Safety ECU] "
        f"ToF worker started "
        f"(PID={process.pid})"
    )

    return process

def main():
    # -------------------------------------------------
    # Modules
    # -------------------------------------------------
    receiver = CANReceiver("can0")
    sender = CANSender("can0")

    state = SystemState()

    logger = PedalLogger(
        "pedal_data.csv"
    )

    system_logger = SystemLogger(
        "system_data.csv"
    )

    failsafe = FailSafeManager()
    command_mapper = CommandMapper()

    bns_calculator = BNSCalculator()
    pms_calculator = PMSCalculator()
    risk_manager = RiskManager()

    # -------------------------------------------------
    # Pedal communication monitoring
    # -------------------------------------------------
    previous_alive_counter = None

    last_pedal_time = time.monotonic()

    pedal_timeout_reported = False

    # -------------------------------------------------
    # Haptic ECU communication monitoring
    # -------------------------------------------------
    haptic_timeout_reported = False

    # -------------------------------------------------
    # ToF process
    # -------------------------------------------------
    tof_queue = multiprocessing.Queue()

    tof_process = None

    last_tof_restart_attempt = 0.0

    # -------------------------------------------------
    # Distance freshness monitoring
    # -------------------------------------------------
    distance_timeout_reported = False

    # -------------------------------------------------
    # Fail-safe monitoring
    # -------------------------------------------------
    previous_failsafe_active = None
    previous_failsafe_reason = None

    # -------------------------------------------------
    # CAN command transmission
    # -------------------------------------------------
    last_command_tx_time = 0.0

    haptic_alive_counter = 0
    motor_alive_counter = 0

    try:
        # -------------------------------------------------
        # Open CAN
        # -------------------------------------------------
        receiver.open()
        sender.open()

        # -------------------------------------------------
        # Start ToF worker
        # -------------------------------------------------
        tof_process = start_tof_process(
            tof_queue
        )

        print("[Safety ECU] Started")

        print(
            "[Safety ECU] "
            "Waiting for PEDAL_STATUS..."
        )

        while True:
            # =================================================
            # CAN receive
            # =================================================
            msg = receiver.receive(
                timeout=0.001
            )

            if msg is not None:

                # ---------------------------------------------
                # PEDAL_STATUS
                # ---------------------------------------------
                if (
                    msg.arbitration_id
                    == PEDAL_STATUS_ID
                ):
                    try:
                        pedal = decode_pedal_status(
                            msg.data
                        )

                        state.update_pedal(
                            pedal
                        )

                        logger.log(
                            state
                        )

                        last_pedal_time = (
                            time.monotonic()
                        )

                        pedal_timeout_reported = (
                            False
                        )

                        current_alive_counter = (
                            pedal["alive_counter"]
                        )

                        if (
                            previous_alive_counter
                            is not None
                        ):
                            expected_alive_counter = (
                                previous_alive_counter
                                + 1
                            ) % 256

                            if (
                                current_alive_counter
                                != expected_alive_counter
                            ):
                                print(
                                    "[PEDAL] "
                                    "ALIVE COUNTER ERROR: "
                                    f"expected="
                                    f"{expected_alive_counter}, "
                                    f"received="
                                    f"{current_alive_counter}"
                                )

                        previous_alive_counter = (
                            current_alive_counter
                        )

                        sensor_errors_text = ",".join(
                            pedal["sensor_errors"]
                        )

                        print(
                            f"[PEDAL] "
                            f"ACC="
                            f"{pedal['accelerator']}% "
                            f"BRAKE="
                            f"{pedal['brake']}% "
                            f"ACC_RATE="
                            f"{pedal['accelerator_rate']:.1f}%/s "
                            f"BRAKE_RATE="
                            f"{pedal['brake_rate']:.1f}%/s "
                            f"STATUS="
                            f"0x{pedal['sensor_status']:02X} "
                            f"ERRORS="
                            f"{sensor_errors_text} "
                            f"ALIVE="
                            f"{pedal['alive_counter']}"
                        )

                    except ValueError as error:
                        print(
                            "[PEDAL] "
                            f"Invalid message: {error}"
                        )

                # ---------------------------------------------
                # HAPTIC_STATUS
                # ---------------------------------------------
                elif (
                    msg.arbitration_id
                    == HAPTIC_STATUS_ID
                ):
                    try:
                        haptic = decode_haptic_status(
                            msg.data
                        )

                        state.update_haptic(
                            haptic
                        )

                        haptic_timeout_reported = (
                            False
                        )

                        print(
                            f"[HAPTIC] "
                            f"RISK="
                            f"{haptic['risk_level']} "
                            f"VIB="
                            f"{haptic['vibration_status']} "
                            f"INTENSITY="
                            f"{haptic['intensity']}% "
                            f"FREQ="
                            f"{haptic['frequency']}Hz "
                            f"FAULT="
                            f"{haptic['fault_code']} "
                            f"CMD_TIMEOUT="
                            f"{haptic['command_timeout']} "
                            f"ECU_STATUS="
                            f"{haptic['ecu_status']} "
                            f"ALIVE="
                            f"{haptic['alive_counter']}"
                        )

                    except ValueError as error:
                        print(
                            "[HAPTIC] "
                            f"Invalid message: {error}"
                        )

            # =================================================
            # Pedal timeout
            # =================================================
            if (
                time.monotonic()
                - last_pedal_time
                >= 0.1
            ):
                state.pedal_connected = False

                if not pedal_timeout_reported:
                    print(
                        "[PEDAL] "
                        "COMMUNICATION TIMEOUT"
                    )

                    pedal_timeout_reported = (
                        True
                    )

            # =================================================
            # Haptic ECU freshness check
            # =================================================
            was_haptic_connected = (
                state.haptic_connected
            )

            state.check_haptic_freshness(
                timeout=0.1
            )

            if (
                was_haptic_connected
                and not state.haptic_connected
            ):
                if not haptic_timeout_reported:
                    print(
                        "[HAPTIC] "
                        "COMMUNICATION TIMEOUT"
                    )

                    haptic_timeout_reported = (
                        True
                    )

            # =================================================
            # Receive data from ToF worker
            # =================================================
            while True:
                try:
                    tof_message = (
                        tof_queue.get_nowait()
                    )

                except queue.Empty:
                    break

                message_type = (
                    tof_message.get("type")
                )

                if message_type == "connected":
                    state.tof_connected = True

                    print(
                        "[ToF] "
                        "Sensor connected"
                    )

                elif message_type == "distance":
                    distance = (
                        tof_message["distance_cm"]
                    )

                    state.update_distance(
                        distance
                    )

                    distance_timeout_reported = (
                        False
                    )

                elif (
                    message_type
                    == "disconnected"
                ):
                    if state.tof_connected:
                        print(
                            "[ToF] "
                            "Sensor disconnected"
                        )

                    state.tof_connected = False

                    state.invalidate_distance()

                    print(
                        f"[ToF STATE] "
                        f"TOF_CONNECTED="
                        f"{state.tof_connected} "
                        f"DIST_VALID="
                        f"{state.distance_valid}"
                    )

            # =================================================
            # Check ToF worker
            # =================================================
            if (
                tof_process is None
                or not tof_process.is_alive()
            ):
                state.tof_connected = False

                state.invalidate_distance()

                current_time = (
                    time.monotonic()
                )

                if (
                    current_time
                    - last_tof_restart_attempt
                    >= 1.0
                ):

                    last_tof_restart_attempt = (
                        current_time
                    )

                    if tof_process is not None:
                        tof_process.join(
                            timeout=0.1
                        )

                    print(
                        "[Safety ECU] "
                        "Restarting ToF worker..."
                    )

                    tof_process = (
                        start_tof_process(
                            tof_queue
                        )
                    )

            # =================================================
            # Distance freshness check
            # =================================================
            was_distance_valid = (
                state.distance_valid
            )

            state.check_distance_freshness(
                timeout=DISTANCE_FRESHNESS_TIMEOUT
            )

            if (
                was_distance_valid
                and not state.distance_valid
                and state.tof_connected
            ):
                if not distance_timeout_reported:
                    print(
                        "[ToF] "
                        "DISTANCE DATA TIMEOUT"
                    )

                    print(
                        f"[ToF STATE] "
                        f"TOF_CONNECTED="
                        f"{state.tof_connected} "
                        f"DIST_VALID="
                        f"{state.distance_valid}"
                    )

                    distance_timeout_reported = (
                        True
                    )

            # =================================================
            # BNS calculation
            # =================================================
            state.bns = (
                bns_calculator.calculate(
                    distance_cm=(
                        state.distance_cm
                    ),
                    distance_valid=(
                        state.distance_valid
                    ),
                    vehicle_speed=(
                        state.vehicle_speed
                    ),
                    ttc=state.ttc,
                )
            )

            # =================================================
            # PMS calculation
            # =================================================
            state.pms = (
                pms_calculator.calculate(
                    accelerator=(
                        state.accelerator
                    ),
                    accelerator_rate=(
                        state.accelerator_rate
                    ),
                    pedal_connected=(
                        state.pedal_connected
                    ),
                    sensor_status=(
                        state.pedal_sensor_status
                    ),
                )
            )

            # =================================================
            # Integrated risk evaluation
            # =================================================
            (
                state.total_risk,
                state.risk_level,
            ) = risk_manager.evaluate(
                bns=state.bns,
                bns_available=(
                    bns_calculator.available
                ),
                pms=state.pms,
                pms_available=(
                    pms_calculator.available
                ),
            )

            # =================================================
            # Fail-safe evaluation
            # =================================================
            failsafe.evaluate(
                state
            )

            if (
                failsafe.active
                != previous_failsafe_active
                or failsafe.reason
                != previous_failsafe_reason
            ):
                if failsafe.active:
                    print(
                        "[FAIL-SAFE] "
                        f"ACTIVE "
                        f"REASON="
                        f"{failsafe.reason}"
                    )

                else:
                    print(
                        "[FAIL-SAFE] "
                        "CLEARED"
                    )

                previous_failsafe_active = (
                    failsafe.active
                )

                previous_failsafe_reason = (
                    failsafe.reason
                )

            # =================================================
            # Haptic / Motor command transmission
            # =================================================
            current_time = time.monotonic()

            if (
                current_time
                - last_command_tx_time
                >= 0.01
            ):
                last_command_tx_time = (
                    current_time
                )

                commands = command_mapper.map(
                    risk_level=(
                        state.risk_level
                    ),
                    risk_available=(
                        risk_manager.available
                    ),
                    failsafe_active=(
                        failsafe.active
                    ),
                )

                # ---------------------------------------------
                # Haptic command
                # ---------------------------------------------
                sender.send_haptic_command(
                    risk_level=(
                        state.risk_level
                    ),
                    vibration_command=(
                        commands["haptic"][
                            "vibration_command"
                        ]
                    ),
                    intensity=(
                        commands["haptic"][
                            "intensity"
                        ]
                    ),
                    frequency=(
                        commands["haptic"][
                            "frequency"
                        ]
                    ),
                    alive_counter=(
                        haptic_alive_counter
                    ),
                )

                # ---------------------------------------------
                # Motor command
                # ---------------------------------------------
                sender.send_motor_command(
                    control_mode=(
                        commands["motor"][
                            "control_mode"
                        ]
                    ),
                    output_limit=(
                        commands["motor"][
                            "output_limit"
                        ]
                    ),
                    risk_level=(
                        state.risk_level
                    ),
                    command_flags=(
                        commands["motor"][
                            "command_flags"
                        ]
                    ),
                    alive_counter=(
                        motor_alive_counter
                    ),
                )

                # ---------------------------------------------
                # System logger
                # ---------------------------------------------
                system_logger.log(
                    state=state,
                    bns_available=(
                        bns_calculator.available
                    ),
                    pms_available=(
                        pms_calculator.available
                    ),
                    risk_available=(
                        risk_manager.available
                    ),
                    failsafe_active=(
                        failsafe.active
                    ),
                    failsafe_reason=(
                        failsafe.reason
                    ),
                    commands=commands,
                )

                haptic_alive_counter = (
                    haptic_alive_counter + 1
                ) % 256

                motor_alive_counter = (
                    motor_alive_counter + 1
                ) % 256

            # =================================================
            # State output
            # =================================================
            if state.distance_valid:
                print(
                    f"[STATE] "
                    f"ACC={state.accelerator}% "
                    f"ACC_RATE="
                    f"{state.accelerator_rate:.1f}%/s "
                    f"DIST="
                    f"{state.distance_cm:.1f}cm "
                    f"PEDAL_CONNECTED="
                    f"{state.pedal_connected} "
                    f"TOF_CONNECTED="
                    f"{state.tof_connected} "
                    f"DIST_VALID="
                    f"{state.distance_valid} "
                    f"BNS="
                    f"{state.bns:.1f} "
                    f"BNS_AVAILABLE="
                    f"{bns_calculator.available} "
                    f"PMS="
                    f"{state.pms:.1f} "
                    f"PMS_AVAILABLE="
                    f"{pms_calculator.available} "
                    f"TOTAL_RISK="
                    f"{state.total_risk:.1f} "
                    f"RISK_LEVEL="
                    f"{state.risk_level} "
                    f"RISK_AVAILABLE="
                    f"{risk_manager.available} "
                    f"HAPTIC_CONNECTED="
                    f"{state.haptic_connected} "
                    f"HAPTIC_AVAILABLE="
                    f"{state.haptic_available} "
                    f"HAPTIC_FAULT="
                    f"{state.haptic_fault_code} "
                    f"HAPTIC_CMD_TIMEOUT="
                    f"{state.haptic_command_timeout}"
                )

    except KeyboardInterrupt:
        print(
            "\n[Safety ECU] Stopped"
        )

    finally:
        # -------------------------------------------------
        # Stop ToF worker
        # -------------------------------------------------
        if (
            tof_process is not None
            and tof_process.is_alive()
        ):
            tof_process.terminate()

            tof_process.join(
                timeout=1.0
            )

        tof_queue.close()

        sender.close()
        receiver.close()


if __name__ == "__main__":
    multiprocessing.set_start_method(
        "spawn"
    )

    main()