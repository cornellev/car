"""Shared Zenoh configuration and process lifecycle for telemetry commands."""

import json
import logging
import os
import signal
import threading
import time
from contextlib import contextmanager

import zenoh

SPI_KEY = "spi_data"
DEMO_KEY = "topic"


def client_config():
    """Connect explicitly to a router; Tailscale does not need multicast."""
    endpoints = [
        endpoint.strip()
        for endpoint in os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447").split(",")
        if endpoint.strip()
    ]
    if not endpoints:
        raise ValueError("ZENOH_CONNECT must contain at least one Zenoh endpoint")
    return zenoh.Config.from_json5(
        json.dumps(
            {
                "mode": "client",
                "connect": {"endpoints": endpoints, "timeout_ms": 10000},
                "scouting": {"multicast": {"enabled": False}},
            }
        )
    )


@contextmanager
def runtime():
    """Close the session on Ctrl-C, Docker stop (SIGTERM), or an exception."""
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    stop = threading.Event()
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
    try:
        with zenoh.open(client_config()) as session:
            yield session, stop
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def run_periodic(callback, period, stop):
    """Schedule with a monotonic clock and skip missed ticks after a stall."""
    deadline = time.monotonic() + period
    while not stop.wait(max(0.0, deadline - time.monotonic())):
        callback()
        deadline += period
        if deadline < time.monotonic():
            deadline = time.monotonic() + period
