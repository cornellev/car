"""Exercise the telemetry contract over TCP using independent Zenoh sessions."""

import json
import logging
import os
import signal
import socket
import struct
import subprocess
import sys
import time
import uuid
from contextlib import ExitStack
from pathlib import Path
from queue import Queue

import pytest
import zenoh

from py_pubsub import publisher_spi
from py_pubsub.latency_reporter import LatencyReporter
from py_pubsub.publisher_member_function import MinimalPublisher
from py_pubsub.subscriber_member_function import MinimalSubscriber
from py_pubsub.subscriber_spi import SpiSubscriber
from py_pubsub.test_spi_publisher import TestPublisher
from py_pubsub.transport import client_config
from read_shm import SENSOR_FMT, SensorShmReader

ROOT = Path(__file__).resolve().parents[3]


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for Zenoh delivery/discovery")


@pytest.fixture
def network(monkeypatch):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{sock.getsockname()[1]}"
    config = zenoh.Config.from_file(ROOT / "config/zenoh-router.json5")
    config.insert_json5("listen/endpoints", json.dumps([endpoint]))
    monkeypatch.setenv("ZENOH_CONNECT", endpoint)
    with ExitStack() as stack:
        router = stack.enter_context(zenoh.open(config))
        publisher = stack.enter_context(zenoh.open(client_config()))
        subscriber = stack.enter_context(zenoh.open(client_config()))
        assert publisher.info.routers_zid() == [router.zid()]
        assert subscriber.info.routers_zid() == [router.zid()]
        yield publisher, subscriber


@pytest.fixture
def sensor_file(monkeypatch):
    # A POSIX SHM file written without Python's resource tracker, just like the
    # host C++ writer. Never touch the real /dev/shm/sensor_shm.
    path = Path("/dev/shm") / f"zenoh-test-{uuid.uuid4().hex}"
    monkeypatch.setattr(
        publisher_spi, "SensorShmReader", lambda: SensorShmReader(path.name)
    )
    yield path
    path.unlink(missing_ok=True)


def test_shm_json_contract_and_late_writer(network, sensor_file):
    publisher_session, subscriber_session = network
    messages = Queue()
    subscriber_session.declare_subscriber("spi_data", messages.put)
    node = publisher_spi.SpiPublisher(publisher_session)
    try:
        wait_for(lambda: node.publisher_.matching_status.matching)
        node.timer_callback()  # Missing shared memory is retried, not fatal.
        assert messages.empty()
        values = (
            123456789,
            2,
            101,
            3.5,
            48.0,
            102,
            2.5,
            -1.5,
            103,
            100.0,
            101.0,
            104,
            102.0,
            103.0,
            105,
            42.5,
            -76.5,
            90.0,
            12.5,
            106,
            2000.0,
            0.5,
            12.25,
        )
        sensor_file.write_bytes(struct.pack("<I", 8) + struct.pack(SENSOR_FMT, *values))
        before_ns = time.time_ns()
        node.timer_callback()
        sample = messages.get(timeout=5)
        after_ns = time.time_ns()
        assert str(sample.key_expr) == "spi_data"
        assert sample.encoding == zenoh.Encoding.APPLICATION_JSON
        data = json.loads(sample.payload.to_string())
        timestamp = data.pop("_t_publish_ns")
        assert before_ns <= timestamp <= after_ns
        assert data == {
            "seq": 8,
            "global_ts": 123456789,
            "errcount": 2,
            "power": {"ts": 101, "current": 3.5, "voltage": 48.0},
            "steering": {"ts": 102, "brake_pressure": 2.5, "turn_angle": -1.5},
            "rpm_front": {"ts": 103, "rpm_left": 100.0, "rpm_right": 101.0},
            "rpm_back": {"ts": 104, "rpm_left": 102.0, "rpm_right": 103.0},
            "gps": {
                "ts": 105,
                "lat": 42.5,
                "long": -76.5,
                "heading": 90.0,
                "speed": 12.5,
            },
            "motor": {"ts": 106, "rpm": 2000.0, "duty_cycle": 0.5},
            "filtered": {"speed": 12.25},
        }
    finally:
        node.close()
    assert not node.reader.available
    assert sensor_file.exists()  # A reader must not unlink the writer's block.


def test_simulated_publisher_contract(network):
    publisher, subscriber = network
    messages = Queue()
    subscriber.declare_subscriber("spi_data", messages.put)
    node = TestPublisher(publisher)
    wait_for(lambda: node.publisher_.matching_status.matching)
    node.timer_callback()
    sample = messages.get(timeout=5)
    assert sample.encoding == zenoh.Encoding.APPLICATION_JSON
    data = json.loads(sample.payload.to_string())
    assert set(data) == {"sensor1", "sensor2"}
    assert all(type(value) is int and 0 <= value <= 1023 for value in data.values())


def test_demo_roundtrip(network, caplog):
    caplog.set_level(logging.INFO)
    publisher, subscriber = network
    MinimalSubscriber(subscriber)
    messages = Queue()
    subscriber.declare_subscriber("topic", messages.put)
    node = MinimalPublisher(publisher)
    wait_for(lambda: node.publisher_.matching_status.matching)
    node.timer_callback()
    node.timer_callback()
    for i in range(2):
        sample = messages.get(timeout=5)
        assert sample.encoding == zenoh.Encoding.TEXT_PLAIN
        assert sample.payload.to_string() == f"Hello World: {i}"
    wait_for(lambda: 'I heard: "Hello World: 1"' in caplog.text)


def test_spi_subscriber_survives_invalid_payloads(network, caplog):
    caplog.set_level(logging.INFO)
    publisher, subscriber = network
    SpiSubscriber(subscriber)
    pub = publisher.declare_publisher("spi_data")
    wait_for(lambda: pub.matching_status.matching)
    pub.put(b"\xff")
    pub.put("not JSON")
    pub.delete()
    pub.put('{"power": {"voltage": 48}}')
    wait_for(lambda: "Received SPI data: {'power': {'voltage': 48}}" in caplog.text)
    assert caplog.text.count("Failed to decode SPI message") == 2


def test_latency_reporter_survives_invalid_payloads(network, caplog, monkeypatch):
    caplog.set_level(logging.INFO)
    publisher, subscriber = network
    LatencyReporter(subscriber)
    pub = publisher.declare_publisher("spi_data")
    wait_for(lambda: pub.matching_status.matching)
    monkeypatch.setattr("py_pubsub.latency_reporter.time.time_ns", lambda: 100_000_000)
    for payload in (
        b"\xff",
        "not JSON",
        "[]",
        "{}",
        '{"_t_publish_ns": "bad"}',
        '{"_t_publish_ns": true}',
    ):
        pub.put(payload)
    pub.delete()
    pub.put('{"_t_publish_ns": 87500000}')
    wait_for(lambda: "delay_ms=12.50" in caplog.text)
    assert caplog.text.count("cannot compute delay") == 4


@pytest.mark.parametrize(
    "command,key",
    [
        ("talker", "spi_data"),
        ("listener", "spi_data"),
        ("publisher_test", "spi_data"),
        ("latency_reporter", "spi_data"),
        ("talkerdemo", "topic"),
        ("listenerdemo", "topic"),
    ],
)
@pytest.mark.parametrize("stop_signal", [signal.SIGTERM, signal.SIGINT])
def test_commands_deliver_and_stop(network, tmp_path, command, key, stop_signal):
    publisher, subscriber = network
    messages = Queue()
    subscriber.declare_subscriber(key, messages.put)
    pub = publisher.declare_publisher(key)
    log_path = tmp_path / "command.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [str(Path(sys.executable).parent / command)],
            stdout=log,
            stderr=log,
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
        )
        try:
            if command in ("publisher_test", "talkerdemo"):
                messages.get(timeout=10)
            elif command == "talker":
                # Exercise the actual CLI without requiring hardware on the host.
                wait_for(
                    lambda: (
                        "Published:" in log_path.read_text()
                        or "Not connected to shared memory" in log_path.read_text()
                    )
                )
            else:
                payload = json.dumps({"_t_publish_ns": time.time_ns()})

                def received():
                    pub.put(payload)
                    return any(
                        text in log_path.read_text()
                        for text in (
                            "Received SPI data:",
                            "delay_ms=",
                            "I heard:",
                        )
                    )

                wait_for(received)
            process.send_signal(stop_signal)
            assert process.wait(timeout=5) == 0, log_path.read_text()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
    assert "Traceback" not in log_path.read_text()


def test_client_config(monkeypatch):
    monkeypatch.delenv("ZENOH_CONNECT", raising=False)
    config = client_config()
    assert json.loads(config.get_json("connect/endpoints")) == ["tcp/127.0.0.1:7447"]
    assert json.loads(config.get_json("mode")) == "client"
    assert json.loads(config.get_json("scouting/multicast/enabled")) is False
    monkeypatch.setenv("ZENOH_CONNECT", "tcp/100.64.0.1:7447, tcp/100.64.0.2:7447")
    assert json.loads(client_config().get_json("connect/endpoints")) == [
        "tcp/100.64.0.1:7447",
        "tcp/100.64.0.2:7447",
    ]
    monkeypatch.setenv("ZENOH_CONNECT", " , ")
    with pytest.raises(ValueError, match="ZENOH_CONNECT"):
        client_config()
