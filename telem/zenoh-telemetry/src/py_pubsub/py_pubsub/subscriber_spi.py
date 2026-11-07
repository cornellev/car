"""Log the JSON telemetry stream."""

import json
import logging

import zenoh

from .transport import SPI_KEY, runtime


class SpiSubscriber:
    def __init__(self, session):
        self.logger = logging.getLogger("spi_subscriber")
        self.subscription = session.declare_subscriber(SPI_KEY, self.listener_callback)

    def listener_callback(self, sample):
        if sample.kind != zenoh.SampleKind.PUT:
            return
        try:
            payload = sample.payload.to_string()
            self.logger.info('I heard: "%s"', payload)
            data = json.loads(payload)
            self.logger.info("Received SPI data: %s", data)
        except ValueError:  # Invalid UTF-8 or JSON.
            self.logger.warning("Failed to decode SPI message")


def main(args=None):
    with runtime() as (session, stop):
        node = SpiSubscriber(session)
        try:
            stop.wait()
        finally:
            node.subscription.undeclare()


if __name__ == "__main__":
    main()
