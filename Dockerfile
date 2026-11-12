# CARLA has no macOS Python client; run the Linux one under amd64 emulation.
# Version must match the server (CARLA.app launches 0.9.15).
FROM --platform=linux/amd64 python:3.8-slim
RUN pip install --no-cache-dir carla==0.9.15
