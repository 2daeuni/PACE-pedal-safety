import can


class CANReceiver:
    def __init__(self, channel="can0"):
        self.channel = channel
        self.bus = None

    def open(self):
        self.bus = can.Bus(
            interface="socketcan",
            channel=self.channel,
        )
        print(f"[CAN] {self.channel} opened")

    def receive(self, timeout=None):
        if self.bus is None:
            raise RuntimeError("CAN bus is not opened")

        return self.bus.recv(timeout=timeout)

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            print(f"[CAN] {self.channel} closed")


if __name__ == "__main__":
    receiver = CANReceiver("can0")

    try:
        receiver.open()
        print("[CAN] Waiting for messages...")

        while True:
            msg = receiver.receive()

            if msg is None:
                continue

            print(
                f"ID=0x{msg.arbitration_id:03X} "
                f"DLC={msg.dlc} "
                f"DATA={msg.data.hex(' ').upper()}"
            )

    except KeyboardInterrupt:
        print("\n[CAN] Stopped by user")

    finally:
        receiver.close()
