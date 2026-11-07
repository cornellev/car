"""Report one-way latency using the existing _t_publish_ns JSON field."""

import json
import logging
import time

import zenoh

from .transport import SPI_KEY, runtime


class LatencyReporter:
    def __init__(self, session):
        self.logger = logging.getLogger("latency_reporter")
        self.sub = session.declare_subscriber(SPI_KEY, self.callback)

    def callback(self, sample):
        if sample.kind != zenoh.SampleKind.PUT:
            return
        now_ns = time.time_ns()
        try:
            data = json.loads(sample.payload.to_string())
        except ValueError:  # Invalid UTF-8 or JSON.
            return
        t_publish_ns = data.get("_t_publish_ns") if isinstance(data, dict) else None
        if type(t_publish_ns) is not int:
            self.logger.warning(
                "Message has no valid '_t_publish_ns'; cannot compute delay."
            )
            return
        delay_ms = (now_ns - t_publish_ns) / 1e6
        self.logger.info("delay_ms=%.2f", delay_ms)


def main(args=None):
    with runtime() as (session, stop):
        node = LatencyReporter(session)
        try:
            stop.wait()
        finally:
            node.sub.undeclare()


if __name__ == "__main__":
    main()
