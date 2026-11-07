#!/bin/sh
set -eu

# Retain the entrypoint filename and Compose service name for existing scripts.
export PYTHONUNBUFFERED=1
if [ "$#" -eq 0 ]; then
    set -- talker
fi
exec "$@"
