import os
import threading
import time

# i love shoving types into an untyped language
from typing import Any, Callable, Optional, TypeVar, cast

import docker
from docker.errors import APIError, DockerException, NotFound
from docker.models.containers import Container
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


app = FastAPI(title="rosbag api")

app.add_middleware(
    CORSMiddleware,
    # i use cors btw
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)
# mutex to hopefully ensure we dont do too many things at once
_bag_lock = threading.RLock()

# docker and ros constants
# they dont need to be declared as env vars because they probably will not ever change
ROSBAG_CONTAINER_NAME = "rosbag"
TAILSCALE_CONTAINER_NAME = "ts-authkey-container"
ROSBAG_IMAGE = "tailscale-ros-telemetry-rosbag"
ROS_SERVICE_LABEL = "com.docker.compose.service=ros"


class HealthResponse(BaseModel):
    ok: bool = True


class ContainerStatus(BaseModel):
    id: str
    name: str
    status: str


class BagActionResponse(ContainerStatus):
    action: str


# gets the docker client
def _docker_client() -> docker.DockerClient:
    base_url = os.environ.get("DOCKER_HOST", "unix:///var/run/docker.sock")
    return docker.DockerClient(base_url=base_url)


def _container_name() -> str:
    return ROSBAG_CONTAINER_NAME


def _ros_domain_id() -> str:
    return os.environ.get("ROS_DOMAIN_ID", "14")


def _tailscale_container_name() -> str:
    return TAILSCALE_CONTAINER_NAME


def _find_rosbag_container(client: docker.DockerClient) -> Container:
    try:
        return cast(Container, client.containers.get(_container_name()))
    except NotFound as exc:
        raise HTTPException(
            status_code=404, detail=f"container not found: {_container_name()}"
        ) from exc


T = TypeVar("T")


def _with_client(fn: Callable[[docker.DockerClient], T]) -> T:
    client = _docker_client()
    try:
        return fn(client)
    except DockerException as exc:
        raise HTTPException(status_code=500, detail=f"docker error: {exc}") from exc
    finally:
        try:
            client.close()
        except Exception:
            pass


def _container_status(container: Container) -> ContainerStatus:
    container.reload()
    return ContainerStatus(
        id=cast(str, container.id),
        name=cast(str, container.name),
        status=cast(str, container.status),
    )


def _api_error_status_code(exc: APIError) -> Optional[int]:
    status_code = getattr(exc, "status_code", None)
    if isinstance(status_code, int):
        return status_code

    response = getattr(exc, "response", None)
    if response is None:
        return None
    return cast(Optional[int], getattr(response, "status_code", None))


def _is_name_conflict(exc: APIError) -> bool:
    status_code = _api_error_status_code(exc)
    explanation = str(getattr(exc, "explanation", exc)).lower()
    return status_code == 409 or "already in use" in explanation


def _wait_until_removed(
    client: docker.DockerClient, name: str, timeout_s: float = 10.0
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            client.containers.get(name)
        except NotFound:
            return
        time.sleep(0.1)
    raise HTTPException(
        status_code=500,
        detail=f"timed out waiting for container removal: {name}",
    )


def _get_tailscale_container(client: docker.DockerClient) -> Container:
    ts_container_name = _tailscale_container_name()

    try:
        return cast(Container, client.containers.get(ts_container_name))
    except NotFound as exc:
        raise HTTPException(
            status_code=500,
            detail=f"tailscale container not found: {ts_container_name}",
        ) from exc


def _discover_workspace_from_container(container: Container) -> Optional[str]:
    mounts = cast(list[dict[str, Any]], container.attrs.get("Mounts", []))
    for mount in mounts:
        if mount.get("Destination") == "/workspace":
            source = mount.get("Source")
            if isinstance(source, str) and source:
                return source
    return None


def _discover_rosbag_workspace(client: docker.DockerClient) -> str:
    try:
        container = cast(Container, client.containers.get(_container_name()))
        container.reload()
        workspace = _discover_workspace_from_container(container)
        if workspace:
            return workspace
    except NotFound:
        pass

    ros_containers = cast(
        list[Container],
        client.containers.list(
            all=True,
            filters={"label": ROS_SERVICE_LABEL},
        ),
    )
    for container in ros_containers:
        container.reload()
        workspace = _discover_workspace_from_container(container)
        if workspace:
            return workspace

    raise HTTPException(
        status_code=500,
        detail=(
            "unable to determine rosbag workspace: create the compose-managed ros "
            "container first"
        ),
    )


def _rosbag_container_kwargs(client: docker.DockerClient) -> dict[str, Any]:
    workspace = _discover_rosbag_workspace(client)
    ts_container = _get_tailscale_container(client)
    return {
        "image": ROSBAG_IMAGE,
        "name": _container_name(),
        "entrypoint": ["/entrypoint-rosbag.sh"],
        "working_dir": "/ros-telemetry",
        "environment": {
            "ROS_DISCOVERY_SERVER": "127.0.0.1:11811",
            "ROS_DOMAIN_ID": _ros_domain_id(),
        },
        "volumes": {
            "/dev/shm": {"bind": "/dev/shm", "mode": "rw"},
            workspace: {"bind": "/workspace", "mode": "rw"},
            f"{workspace}/bags": {"bind": "/workspace/bags", "mode": "rw"},
        },
        "network_mode": f"container:{ts_container.id}",
        "restart_policy": {"Name": "unless-stopped"},
    }


def _remove_rosbag_container_if_present(client: docker.DockerClient) -> None:
    name = _container_name()
    try:
        stale = cast(Container, client.containers.get(name))
        stale.reload()
        try:
            if stale.status == "running":
                stale.stop(timeout=20)
        except APIError as exc:
            if _api_error_status_code(exc) not in (304, 404):
                raise
        try:
            stale.remove(force=True)
        except NotFound:
            pass
        _wait_until_removed(client, name)
    except NotFound:
        pass


def _create_rosbag_container(client: docker.DockerClient) -> Container:
    _remove_rosbag_container_if_present(client)

    try:
        return cast(
            Container, client.containers.create(**_rosbag_container_kwargs(client))
        )
    except APIError as exc:
        if _is_name_conflict(exc):
            return cast(Container, client.containers.get(_container_name()))
        raise


@app.get("/healthz", response_model=HealthResponse)
def healthz() -> HealthResponse:
    return HealthResponse()


@app.get("/bag/status", response_model=ContainerStatus)
def bag_status() -> ContainerStatus:
    def _handler(client: docker.DockerClient) -> ContainerStatus:
        container = _find_rosbag_container(client)
        return _container_status(container)

    return _with_client(_handler)


@app.post("/bag/start", response_model=BagActionResponse)
def bag_start() -> BagActionResponse:
    def _handler(client: docker.DockerClient) -> BagActionResponse:
        with _bag_lock:
            try:
                container = cast(Container, client.containers.get(_container_name()))
                container.reload()
                if container.status == "running":
                    status = _container_status(container)
                    return BagActionResponse(**status.model_dump(), action="noop")
            except NotFound:
                pass

            # recreate rather than restart so each start gets a clean bag path and config
            container = _create_rosbag_container(client)
            try:
                container.start()
            except APIError as exc:
                if not _is_name_conflict(exc):
                    raise
                container = cast(Container, client.containers.get(_container_name()))
                container.reload()
                if container.status != "running":
                    raise
                status = _container_status(container)
                return BagActionResponse(**status.model_dump(), action="noop")
            status = _container_status(container)
            return BagActionResponse(**status.model_dump(), action="started")

    return _with_client(_handler)


@app.post("/bag/stop", response_model=BagActionResponse)
def bag_stop() -> BagActionResponse:
    def _handler(client: docker.DockerClient) -> BagActionResponse:
        with _bag_lock:
            try:
                container = cast(Container, client.containers.get(_container_name()))
                container.reload()
            except NotFound:
                # nothing to stop — treat as noop
                return BagActionResponse(
                    id="",
                    name=_container_name(),
                    status="not found",
                    action="noop",
                )
            # was already not running, ie stopped, so no need to do anything
            if container.status != "running":
                status = _container_status(container)
                return BagActionResponse(**status.model_dump(), action="noop")
            # 20 second timeout should be more than enough
            container.stop(timeout=20)
            status = _container_status(container)
            return BagActionResponse(**status.model_dump(), action="stopped")

    return _with_client(_handler)
