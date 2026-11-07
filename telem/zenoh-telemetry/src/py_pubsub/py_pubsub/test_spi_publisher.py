"""Publish simulated sensors without a shared-memory writer."""

import json
import logging
import random

import zenoh

from .transport import SPI_KEY, run_periodic, runtime


class TestPublisher:
    __test__ = False  # This is a command, not a pytest test case.

    def __init__(self, session):
        self.publisher_ = session.declare_publisher(
            SPI_KEY,
            encoding=zenoh.Encoding.APPLICATION_JSON,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )
        self.logger = logging.getLogger("test_publisher")

    def timer_callback(self):
        snap = {
            "sensor1": random.randint(0, 1023),
            "sensor2": random.randint(0, 1023),
        }
        self.publisher_.put(json.dumps(snap))
        self.logger.info("Published: %s", snap)


def main(args=None):
    with runtime() as (session, stop):
        node = TestPublisher(session)
        try:
            run_periodic(node.timer_callback, 0.1, stop)
        finally:
            node.publisher_.undeclare()


if __name__ == "__main__":
    main()
