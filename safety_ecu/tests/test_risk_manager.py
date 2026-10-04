import os
import sys

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__)
        )
    )
)

from risk.risk_manager import RiskManager


def assert_equal(actual, expected, message):
    if actual != expected:
        raise AssertionError(
            f"{message}: "
            f"expected={expected}, actual={actual}"
        )


def test_two_of_three():
    rm = RiskManager()

    candidate = rm.determine_candidate_level(
        level1_conditions=(True, False, True),
        level2_conditions=(False, False, False),
        level3_conditions=(False, False, False),
    )

    assert_equal(
        candidate,
        1,
        "2-of-3 Level 1 failed",
    )

    print("[PASS] 2-of-3 Level 1")


def test_highest_level_priority():
    rm = RiskManager()

    candidate = rm.determine_candidate_level(
        level1_conditions=(True, True, True),
        level2_conditions=(True, True, True),
        level3_conditions=(True, False, True),
    )

    assert_equal(
        candidate,
        3,
        "Highest Risk Level priority failed",
    )

    print("[PASS] Highest level priority")


def test_single_condition_not_enough():
    rm = RiskManager()

    candidate = rm.determine_candidate_level(
        level1_conditions=(True, False, False),
        level2_conditions=(False, False, False),
        level3_conditions=(False, False, False),
    )

    assert_equal(
        candidate,
        0,
        "Single condition should not trigger risk",
    )

    print("[PASS] Single condition rejected")

def test_upward_transition():
    rm = RiskManager()

    changed = rm.update_candidate_confirmation(2)
    assert_equal(changed, False, "Upward count 1")
    assert_equal(rm.risk_level, 0, "Risk Level count 1")

    changed = rm.update_candidate_confirmation(2)
    assert_equal(changed, False, "Upward count 2")
    assert_equal(rm.risk_level, 0, "Risk Level count 2")

    changed = rm.update_candidate_confirmation(2)
    assert_equal(changed, True, "Upward count 3")
    assert_equal(rm.risk_level, 2, "Risk Level should rise to 2")

    print("[PASS] 3-count upward transition")


def test_candidate_change_resets_count():
    rm = RiskManager()

    rm.update_candidate_confirmation(2)
    rm.update_candidate_confirmation(2)

    assert_equal(
        rm.candidate_count,
        2,
        "Candidate count before change",
    )

    rm.update_candidate_confirmation(1)

    assert_equal(
        rm.candidate_count,
        1,
        "Candidate count reset after change",
    )

    print("[PASS] Candidate change resets count")


def test_downward_transition():
    rm = RiskManager()

    for _ in range(3):
        rm.update_candidate_confirmation(3)

    assert_equal(
        rm.risk_level,
        3,
        "Failed to reach Level 3",
    )

    for _ in range(4):
        changed = rm.update_candidate_confirmation(0)

    assert_equal(
        changed,
        False,
        "Downward transition occurred too early",
    )

    assert_equal(
        rm.risk_level,
        3,
        "Risk Level should remain 3 before 5th count",
    )

    changed = rm.update_candidate_confirmation(0)

    assert_equal(
        changed,
        True,
        "5-count downward transition failed",
    )

    assert_equal(
        rm.risk_level,
        2,
        "Downward transition must decrease one level only",
    )

    print("[PASS] 5-count downward transition")

def test_one_level_down_only():
    rm = RiskManager()

    for _ in range(3):
        rm.update_candidate_confirmation(3)

    for _ in range(5):
        rm.update_candidate_confirmation(0)

    assert_equal(
        rm.risk_level,
        2,
        "First downward transition should end at Level 2",
    )

    for _ in range(5):
        rm.update_candidate_confirmation(0)

    assert_equal(
        rm.risk_level,
        1,
        "Second downward transition should end at Level 1",
    )

    print("[PASS] One-level-at-a-time downward transition")


def test_unavailable_reset():
    rm = RiskManager()

    rm.update_candidate_confirmation(2)
    rm.update_candidate_confirmation(2)

    assert_equal(
        rm.candidate_count,
        2,
        "Pre-reset candidate count",
    )

    rm.evaluate(
        bns=0.0,
        bns_available=False,
        pms=0.0,
        pms_available=True,
    )

    assert_equal(
        rm.available,
        False,
        "Risk availability reset failed",
    )

    assert_equal(
        rm.candidate_level,
        0,
        "Candidate Level reset failed",
    )

    assert_equal(
        rm.candidate_count,
        0,
        "Candidate count reset failed",
    )

    assert_equal(
        rm.risk_level,
        0,
        "Risk Level reset failed",
    )

    print("[PASS] Unavailable reset")


def run_all_tests():
    print("=== RiskManager Tests ===")

    test_two_of_three()
    test_highest_level_priority()
    test_single_condition_not_enough()
    test_upward_transition()
    test_candidate_change_resets_count()
    test_downward_transition()
    test_one_level_down_only()
    test_unavailable_reset()

    print("=========================")
    print("[PASS] ALL RISK MANAGER TESTS")

if __name__ == "__main__":
    run_all_tests()
