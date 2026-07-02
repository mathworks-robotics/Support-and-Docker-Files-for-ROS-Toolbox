#!/bin/bash
# Start sshd on port 22 if it is free; otherwise fall back to port 2222.
#
# The containers in this repository are meant to be run with --net=host so that
# ROS 2 / DDS discovery works between MATLAB on the host and the ROS nodes in the
# container. Because the container shares the host network namespace, a listener
# on the host's port 22 (e.g. the host's own sshd on a Linux PC) is visible from
# inside the container. In that case we bind sshd to 2222 instead of failing.
#
# The chosen port is printed below and is visible via `docker logs <container>`.

SSH_PORT=22

# `/dev/tcp` is a bash built-in, so this check needs no extra packages.
# A successful connection means something is already listening on port 22.
if (echo > /dev/tcp/127.0.0.1/22) 2>/dev/null; then
    echo "[ssh-fallback] Port 22 already in use on host - falling back to 2222"
    SSH_PORT=2222
else
    echo "[ssh-fallback] Port 22 is free - using 22"
fi

echo "[ssh-fallback] Starting sshd on port ${SSH_PORT}"
# -D keeps sshd in the foreground so supervisord can manage it properly.
exec /usr/sbin/sshd -D -p "${SSH_PORT}"
