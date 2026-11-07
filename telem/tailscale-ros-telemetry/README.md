# tailscale-ros-telemetry

reading shm telemetry data and publishing as ROS2 topics

## how it works

- the host writes telemetry data to a shared memory
- we read the telemetry data from shared memory and publish it as the ROS2 topic `/spi_data`
- the ROS2 node communicates over a Tailscale network, allowing subscribers to receive data as well

## project structure

```
docker-compose.yml      # service definitions
config/                 # env files and XML profiles
docker/                 # Dockerfiles and entrypoint scripts
src/                    # ROS2 package source
```

## setup

copy `.env.example` to `.env` and fill in your values:

```bash
cp .env.example .env
```

| Variable | Required | Default | Description |
|---|---|---|---|
| `TS_CLIENT_ID` | yes | — | Tailscale OAuth client ID |
| `TS_CLIENT_SECRET` | yes | — | Tailscale OAuth client secret |
| `ROS_DOMAIN_ID` | no | `14` | ROS2 domain ID |
| `ROS_DISCOVERY_SERVER` | no | `127.0.0.1:11811` | Fast DDS discovery server address |
| `ROSBAG_API_PORT` | no | `8080` | Port the rosbag HTTP API listens on |
| `GPIO_DEVICE` | no | `/dev/gpiomem` | GPIO device path (Raspberry Pi 5: `/dev/gpiomem4`) |

## docker compose services

there are six services defined in [docker-compose.yml](docker-compose.yml):

### `tailscale`

Connects the device to your Tailscale network and gates startup for the other services

- authenticates using OAuth credentials (`TS_CLIENT_ID` / `TS_CLIENT_SECRET`) and advertises the tag `tag:ros-device`
- runs [entrypoint-tailscale.sh](docker/entrypoint-tailscale.sh), which waits for the Tailscale connection to be established before exiting the startup loop

### `discovery-server`

Runs a [Fast DDS Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html), which centralizes DDS peer discovery instead of using multicast/unicast to all known peers.

- runs [entrypoint-discovery.sh](docker/entrypoint-discovery.sh): sources the ROS2 environment and starts `fastdds discovery --server-id 0` on port 11811 (UDP)
- shares the `tailscale` network namespace, so it is reachable on the Tailscale IP of this device at port 11811
- `ros` connects as a plain CLIENT via `ROS_DISCOVERY_SERVER=127.0.0.1:11811`
- `rosbag` connects as a SUPER_CLIENT via an XML profile

### `ros`

The main ROS2 publisher. Reads telemetry from host shared memory and publishes to the `/spi_data` topic.

- runs [entrypoint-ros.sh](docker/entrypoint-ros.sh): sources the ROS2 environment and launches the `py_pubsub talker` node
- runs as a plain DDS CLIENT via `ROS_DISCOVERY_SERVER=127.0.0.1:11811`
- binds and mounts `/dev/shm` from the host to:
    - read telemetry data from host
    - communicate with `rosbag`, as FastDDS uses shared memory when possible
- accesses GPIO via the configured `GPIO_DEVICE` (default `/dev/gpiomem`)

### `rosbag`

Records the `/spi_data` topic to timestamped bag files. This service does not
start immediately with the others (profile `bag`), but is controlled via the
`rosbag-api` service.

- runs [entrypoint-rosbag.sh](docker/entrypoint-rosbag.sh): sources the ROS2 environment and runs `ros2 bag record /spi_data`
- runs as a SUPER_CLIENT — required for `ros2 bag record` to discover topic types
- binds and mounts `/dev/shm` from the host for FastDDS shared memory transport with `ros`
- saves bags to `./bags/bag_YYYYMMDD-HHMMSS/`
- starts after the `ros` service has started

To create the container without starting it (so the API can start/stop it):

```bash
docker compose --profile bag up -d --no-start rosbag
```

To start it immediately without the API:

```bash
docker compose --profile bag up -d rosbag
```

### `rosbag-api`

Super simple HTTP API that starts/stops the rosbag container. It is intended for
a frontend to control recording without needing direct access to Docker or the command line.

- talks to Docker via `/var/run/docker.sock`
- exposed on port `8080` on the device's Tailscale IP (configurable via `ROSBAG_API_PORT`)

Endpoints:

- `POST /bag/start`: Starts the `rosbag` container
- `POST /bag/stop`: Stops the `rosbag` container
- `GET /bag/status`: Returns whether the `rosbag` container is running or not
- `GET /healthz`: Healthcheck endpoint

Optional environment variables (all configurable via `.env`):

- `ROSBAG_API_PORT` (default `8080`) — port the API listens on

The rosbag API uses fixed internal values for the rosbag container name, the
Tailscale container name, and the rosbag image. It discovers the workspace bind
mount from the compose-managed containers instead of reading it from the
environment.

Example:

```bash
curl -X POST http://<tailscale-ip>:8080/bag/start
curl http://<tailscale-ip>:8080/bag/status
curl -X POST http://<tailscale-ip>:8080/bag/stop
```

### `subscriber`


A verification listener that subscribes to `/spi_data` and logs received messages. Hidden behind the `verify` profile so it doesn't start by default.

- runs [entrypoint-subscriber.sh](docker/entrypoint-subscriber.sh): sources the ROS2 environment and launches the `py_pubsub listener` node
- runs as a plain DDS CLIENT via `ROS_DISCOVERY_SERVER` (defaults to `127.0.0.1:11811`, overridable via env var)
- start locally: `docker compose --profile verify up -d`
- can also run on a remote machine (see [using this repo on a remote machine](#using-this-repo-on-a-remote-machine))

## shared memory and discovery

`rosbag` service needs two special things:
- shm
    - fastDDS use shared memory transport for same-host clients, so we need to bind to `/dev/shm` to allow data to flow
- SUPER_CLIENT
    - `ros bag record` requires introspection in order to know the type of a topic. (not possible to manually specify it like you do for `ros topic echo`)
    - with a discovery server, only a SUPER_CLIENT can use introspection

### startup order

the services are started sequentially, one after the other

1. `tailscale`: connects to the tailnet
2. `discovery-server`: starts the Fast DDS discovery server
3. `ros`: starts publishing
4. `rosbag`: starts recording

## How to add a subscriber

Subscribers connect to the discovery server over the Tailscale network. Find the publisher's Tailscale IP:

```bash
docker exec ts-authkey-container tailscale ip -4
```

### Using this repo on a remote machine

If this repo is cloned on the remote machine:

```bash
cd tailscale-ros-telemetry
ROS_DISCOVERY_SERVER=<publisher-tailscale-ip>:11811 docker compose --profile verify up -d tailscale subscriber
docker compose --profile verify logs -f subscriber
```

### Adding a docker subscriber

```yaml
services:
  tailscale:
    image: tailscale/tailscale:latest
    container_name: ts-subscriber
    hostname: my-subscriber
    environment:
      - TS_CLIENT_ID=${TS_CLIENT_ID:-}
      - TS_CLIENT_SECRET=${TS_CLIENT_SECRET:-}
      - TS_STATE_DIR=/var/lib/tailscale
      - TS_USERSPACE=false
      - TS_AUTH_ONCE=true
      - TS_EXTRA_ARGS=--advertise-tags=tag:ros-device --ssh=true
    volumes:
      - ts-authkey-container:/var/lib/tailscale
      - /dev/net/tun:/dev/net/tun
    cap_add:
      - NET_ADMIN
      - NET_RAW

  subscriber:
    image: ros:humble-ros-base-jammy
    network_mode: service:tailscale
    environment:
      - ROS_DISCOVERY_SERVER=<publisher-tailscale-ip>:11811
    command: bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /spi_data std_msgs/msg/String"

volumes:
  ts-authkey-container:
    driver: local
```

### SUPER_CLIENT mode

Required for introspection tools (`ros2 topic list`, `ros2 node list`, `ros2 bag record`). Generate the XML profile and mount it:

```bash
sed 's/DISCOVERY_SERVER_IP/<publisher-tailscale-ip>/' config/super_client.example.xml > /tmp/super_client.xml
```

```yaml
  your_service:
    network_mode: service:tailscale
    environment:
      - FASTRTPS_DEFAULT_PROFILES_FILE=/config/super_client.xml
    volumes:
      - /tmp/super_client.xml:/config/super_client.xml:ro
```
