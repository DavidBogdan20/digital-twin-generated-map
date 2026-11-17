# CARLA has no macOS Python client; run the Linux one under amd64 emulation. The client must match the server:
# 0.9.16 by default (it needs Python 3.10+). For the Mac's CARLA.app, which is still 0.9.15:
#   docker build --build-arg PY=3.8 --build-arg CARLA=0.9.15 -t carla-client:0.9.15 .
ARG PY=3.10
FROM --platform=linux/amd64 python:${PY}-slim
ARG CARLA=0.9.16
RUN pip install --no-cache-dir carla==${CARLA}
