# tailscale-ros-telemetry

Read host shared-memory telemetry and publish it as JSON over Zenoh and Tailscale.
The live publisher, subscribers, demos, and latency reporter run without ROS.
ROS bag recording is retained as legacy tooling and has **not** been migrated.

## Data interface

| Former ROS topic | Zenoh key | Payload | Publisher rate |
|---|---|---|---|
| `/spi_data` | `spi_data` | UTF-8 JSON, `application/json` | ~50 Hz |
| `/topic` | `topic` | UTF-8 text, `text/plain` (`Hello World: N`) | 2 Hz |

The telemetry payload is exactly the JSON previously stored in `std_msgs/String.data`.
There is no new wrapper: `seq`, `global_ts`, `errcount`, `power`, `steering`,
`rpm_front`, `rpm_back`, `gps`, `motor`, `filtered`, and their nested fields keep
the same names, values, and units. `_t_publish_ns` remains an integer Unix-epoch
publication timestamp in nanoseconds. The shared-memory reader and its binary
layout are unchanged.

Zenoh keys omit the ROS leading slash. Existing ROS/DDS clients need to switch
their subscription to Zenoh; their JSON parsing and sensor processing can stay
largely the same:

```python
# Before, in a ROS subscription callback:
data = json.loads(msg.data)

# Now, in a Zenoh subscription callback:
data = json.loads(sample.payload.to_string())
```

Publications use blocking congestion control over TCP. This is a live stream:
there is no replay, retained last value, or guarantee of delivery during a
connection outage. ROS QoS queue depth, domain IDs, remapping, and simulated ROS
time do not apply. Use separate routers or distinct keys if multiple publishers
need isolated streams.

## Setup

This project lives in the monorepo at `telem/zenoh-telemetry`. The publisher
reads shared memory with [`telem/shm/read_shm.py`](../shm/read_shm.py), which
Compose passes to the image as the `shm` build context (Docker Compose 2.17+).
Run the commands below from `telem/zenoh-telemetry`. Configure Tailscale:

```bash
cp .env.example .env
```

| Variable | Default | Description |
|---|---|---|
| `TS_CLIENT_ID` | required | Tailscale OAuth client ID |
| `TS_CLIENT_SECRET` | required | Tailscale OAuth client secret |
| `ZENOH_CONNECT` | `tcp/127.0.0.1:7447` | Router endpoint; comma-separated endpoints are also supported |
| `ROS_DOMAIN_ID` | `14` | Legacy bag tooling only |
| `ROSBAG_API_PORT` | `8080` | Legacy bag HTTP API only |

Start the host sensor writer (see [telem/shm](../shm/README.md)). It owns `/dev/shm/sensor_shm`; the
publisher only reads it and retries attaching if the writer has not started.
The Python publisher does not require GPIO access or pigpio.

```bash
docker compose up -d --build
docker compose logs -f subscriber
```

The default services are:

- `tailscale`: joins the tailnet using the existing hostname and `tag:ros-device`.
- `zenoh-router`: runs Zenoh 1.9.0 on TCP port 7447 inside the Tailscale network
  namespace, using [config/zenoh-router.json5](config/zenoh-router.json5).
- `ros`: the Zenoh telemetry publisher. The old service name is kept for scripts
  and the legacy bag API's workspace lookup. It mounts the host `/dev/shm`.
- `subscriber`: logs JSON received on `spi_data`, using the same Python image.

The router starts after Tailscale is healthy; the publisher starts after the
router container starts. Clients allow up to 10 seconds to establish the initial
connection, and Compose restarts failed processes. Multicast discovery is
disabled; all clients connect explicitly to the router. Only TCP port 7447 needs
to be reachable through the tailnet for live telemetry. There is no published
Docker host port.

The `ros` service still mounts the workspace for compatibility, but Python code
is installed in the image. Rebuild the image after changing the source.

## Remote subscribers

Find the publisher's Tailscale IP:

```bash
docker exec ts-authkey-container tailscale ip -4
```

On a remote machine with a clone of this monorepo and its own Tailscale
credentials in `.env`, from `telem/zenoh-telemetry`:

```bash
ZENOH_CONNECT=tcp/<publisher-tailscale-ip>:7447 \
  docker compose up -d --build tailscale subscriber
docker compose logs -f subscriber
```

Starting just `tailscale subscriber` does not start a local router or publisher.
The subscriber doesn't need access to the publisher's shared memory.

A standalone Python client on a machine already connected to Tailscale only
needs `pip install eclipse-zenoh==1.9.0`:

```python
import json
import time
import zenoh

config = zenoh.Config.from_json5(json.dumps({
    "mode": "client",
    "connect": {"endpoints": ["tcp/<publisher-tailscale-ip>:7447"]},
    "scouting": {"multicast": {"enabled": False}},
}))


def on_data(sample):
    if sample.kind == zenoh.SampleKind.PUT:
        data = json.loads(sample.payload.to_string())
        print(data)


try:
    with zenoh.open(config) as session:
        subscriber = session.declare_subscriber("spi_data", on_data)
        while True:
            time.sleep(1)
except KeyboardInterrupt:
    pass
```

See the [official Zenoh Python API documentation](https://zenoh-python.readthedocs.io/)
for other client operations.

## Local development and commands

Use Python 3.10 or newer. `nix develop` optionally supplies Python and virtualenv;
ROS is no longer needed in the default development shell.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e 'src/py_pubsub[test]'
```

Run a local router (or point `ZENOH_CONNECT` at an existing one):

```bash
docker run --rm -p 127.0.0.1:7447:7447 \
  -v "$PWD/config/zenoh-router.json5:/config/zenoh-router.json5:ro" \
  eclipse/zenoh:1.9.0 --config /config/zenoh-router.json5
```

The former `ros2 run py_pubsub <command>` commands are installed directly into
the Python environment:

| Command | Behavior |
|---|---|
| `talker` | Publish shared-memory telemetry on `spi_data` at ~50 Hz |
| `listener` | Log the JSON telemetry stream |
| `publisher_test` | Publish the existing random `sensor1`/`sensor2` JSON at 10 Hz |
| `latency_reporter` | Log `delay_ms` from `_t_publish_ns` |
| `talkerdemo` | Publish `Hello World: N` on `topic` at 2 Hz |
| `listenerdemo` | Log messages from `topic` |

For example, run `publisher_test` and `listener` in separate terminals for a
hardware-free smoke test. The simulated publisher retains its two-field payload
and does not include `_t_publish_ns`; use `talker` to measure sensor latency.
Clocks must be synchronized between machines for meaningful one-way latency.
All commands close their Zenoh sessions on SIGINT or SIGTERM.

You can also run commands in the existing Compose image:

```bash
docker compose run --rm ros latency_reporter
```

Tests start an isolated local router and use actual TCP pub/sub, including a
temporary shared-memory sensor block. No Tailscale credentials or hardware are
needed:

```bash
python -m pytest src/py_pubsub/test -q
```

## Deferred bag recording

`rosbag`, `rosbag-api`, and the old DDS `discovery-server` are kept behind the
`bag` profile. Their ROS recording/control behavior remains in place, including
`POST /bag/start`, `POST /bag/stop`, `GET /bag/status`, and `GET /healthz`.
They do **not** receive the new Zenoh stream, and `/bag/start` does not imply that
Zenoh telemetry is being recorded. No ROS-to-Zenoh bridge or new recording format
is included in this migration.

The legacy ROS Dockerfile, SUPER_CLIENT XML, bag storage paths, and API are kept
for that follow-up work. `nix develop .#rosbag` retains the legacy ROS shell.
