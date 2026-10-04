import os
import sys

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__)
        )
    )
)

from control.failsafe import FailSafeManager
from control.state import SystemState


def assert_equal(actual, expected, message):
    if actual != expected:
        raise AssertionError(
            f"{message}: expected={expected}, actual={actual}"
        )


def test_tof_failure():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = False
    state.pedal_connected = False
    state.pedal_sensor_status = None

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        True,
        "ToF failure should activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        "TOF_COMMUNICATION_ERROR",
        "Wrong fail-safe reason for ToF failure",
    )

    print("[PASS] ToF communication failure")


def test_normal_state():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = False
    state.pedal_sensor_status = None

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        False,
        "Normal state should not activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        None,
        "Normal state should clear fail-safe reason",
    )

    print("[PASS] Normal state")


def test_pedal_sensor_error():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = True
    state.pedal_sensor_status = 0x01

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        True,
        "Pedal sensor error should activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        "PEDAL_SENSOR_ERROR",
        "Wrong fail-safe reason for pedal sensor error",
    )

    print("[PASS] Pedal sensor error")


def test_pedal_disconnected_not_yet_failsafe():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = False
    state.pedal_sensor_status = None

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        False,
        "Pedal disconnect should not activate fail-safe yet",
    )

    print("[PASS] Pedal disconnect currently ignored")


def test_recovery():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = False
    state.pedal_connected = False
    state.pedal_sensor_status = None

    failsafe.evaluate(state)

    assert_equal(
        failsafe.active,
        True,
        "Fail-safe activation before recovery failed",
    )

    state.tof_connected = True

    failsafe.evaluate(state)

    assert_equal(
        failsafe.active,
        False,
        "Fail-safe did not clear after recovery",
    )

    assert_equal(
        failsafe.reason,
        None,
        "Fail-safe reason did not clear after recovery",
    )

    print("[PASS] Fail-safe recovery")


def run_all_tests():
    print("=== FailSafeManager Tests ===")

    test_tof_failure()
    test_normal_state()
    test_pedal_sensor_error()
    test_pedal_disconnected_not_yet_failsafe()
    test_recovery()

    print("=============================")
    print("[PASS] ALL FAIL-SAFE TESTS")


if __name__ == "__main__":
    run_all_tests()
