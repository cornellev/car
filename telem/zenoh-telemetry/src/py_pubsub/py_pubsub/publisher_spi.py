"""Publish the existing shared-memory snapshot as UTF-8 JSON over Zenoh."""

import json
import logging
import time

import zenoh

from .transport import SPI_KEY, run_periodic, runtime
from read_shm import SensorShmReader


class SpiPublisher:
    def __init__(self, session):
        self.publisher_ = session.declare_publisher(
            SPI_KEY,
            encoding=zenoh.Encoding.APPLICATION_JSON,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )
        self.reader = SensorShmReader()
        self.logger = logging.getLogger("spi_publisher")

    def timer_callback(self):
        if not self.reader.available:
            self.reader.close()
            self.reader = SensorShmReader()
            if not self.reader.available:
                self.logger.warning(
                    "Not connected to shared memory. Is shm writer running?"
                )
                return

        snap = self.reader.read_snapshot_dict()
        if snap:
            # Same Unix-epoch nanoseconds as the former default ROS clock.
            snap["_t_publish_ns"] = time.time_ns()
            payload = json.dumps(snap)
            self.publisher_.put(payload)
            self.logger.info("Published: %s", payload)
        else:
            self.logger.warning("Failed to read snapshot from shared memory")

    def close(self):
        self.reader.close()
        self.publisher_.undeclare()


def main(args=None):
    with runtime() as (session, stop):
        node = SpiPublisher(session)
        try:
            run_periodic(node.timer_callback, 0.02, stop)  # ~50 Hz
        finally:
            node.close()


if __name__ == "__main__":
    main()
