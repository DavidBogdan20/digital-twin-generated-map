# Image for the offline checks that mirrors the VM's "carla" container: Ubuntu 20.04, Python 3.8.10, pip 20.0.2, user
# carla, the carla 0.9.15 wheel and CARLA 0.9.15's agents package at the VM's paths, set up by scripts/setup_env_linux.sh.
# From the repo root (ScenarioRunner v0.9.15 is cloned into external/scenario_runner by the setup if missing):
#
#     docker build --platform linux/amd64 -f scripts/test_image.Dockerfile -t carsim-test:0.9.15 .

FROM --platform=linux/amd64 python:3.8-slim AS fetch
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates && rm -rf /var/lib/apt/lists/*
# The PyPI wheel under the VM's file name (pip 20.0.2 does not know its manylinux_2_27 tag; the binary is the same build)
RUN pip download --no-deps --only-binary=:all: -d /w carla==0.9.15 \
    && mv /w/carla-0.9.15-cp38-*.whl /w/carla-0.9.15-cp38-cp38-linux_x86_64.whl
RUN git init -q /c && cd /c && git remote add origin https://github.com/carla-simulator/carla.git \
    && git config core.sparseCheckout true && echo PythonAPI/carla/agents/ > .git/info/sparse-checkout \
    && git fetch -q --depth 1 origin tag 0.9.15 && git checkout -q 0.9.15

FROM --platform=linux/amd64 ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive SDL_VIDEODRIVER=dummy
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-pip git ca-certificates \
    && rm -rf /var/lib/apt/lists/* && useradd -m carla && mkdir -p /work && chown carla /work
COPY --from=fetch --chown=carla /w/ /work/carla/PythonAPI/carla/dist/
COPY --from=fetch --chown=carla /c/PythonAPI/carla/agents /work/stock/PythonAPI/carla/agents
COPY --chown=carla . /work/car_simulator
USER carla
WORKDIR /work/car_simulator
RUN bash scripts/setup_env_linux.sh
