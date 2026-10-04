import time

from sensors.tof import ToFSensor


def tof_worker(data_queue):

    tof = None

    try:
        print("[ToF Worker] Starting...")

        # ---------------------------------------------
        # Initialize ToF sensor
        # ---------------------------------------------
        tof = ToFSensor(filter_size=5)

        tof.start()

        print("[ToF Worker] Sensor ready")

        # Notify main process
        data_queue.put({
            "type": "connected"
        })

        communication_error_start = None

        while True:
            distance, communication_ok = (
                tof.read_distance()
            )

            # -----------------------------------------
            # Communication OK
            # -----------------------------------------
            if communication_ok:
                communication_error_start = None

                # Send valid distance
                if distance is not None:
                    data_queue.put({
                        "type": "distance",
                        "distance_cm": distance,
                    })

            # -----------------------------------------
            # Communication failure
            # -----------------------------------------
            else:
                if communication_error_start is None:
                    communication_error_start = (
                        time.monotonic()
                    )

                    print(
                        "[ToF Worker] "
                        "Communication error"
                    )

                error_duration = (
                    time.monotonic()
                    - communication_error_start
                )

                # Failure persisted for 0.5 seconds
                if error_duration >= 0.5:
                    print(
                        "[ToF Worker] "
                        "Sensor timeout"
                    )

                    data_queue.put({
                        "type": "disconnected"
                    })

                    print(
                        "[ToF Worker] "
                        "Exiting for restart"
                    )

                    return

            time.sleep(0.02)

    except (OSError, RuntimeError, ValueError) as error:
        print(
            f"[ToF Worker] Sensor error: "
            f"{type(error).__name__}: {error}"
        )

        try:
            data_queue.put({
                "type": "disconnected"
            })

        except Exception:
            pass

    finally:
        if tof is not None:
            tof.stop()

        print("[ToF Worker] Stopped")
