# Image for the offline checks that mirrors the target container: Ubuntu 22.04, Python 3.10, user carla, CARLA 0.9.16's
# agents package at /work/stock (CARLA_ROOT), set up by scripts/setup_env_linux.sh (carla 0.9.16 from PyPI).
# From the repo root (ScenarioRunner v0.9.16 is cloned into external/scenario_runner by the setup if missing):
#
#     docker build --platform linux/amd64 -f scripts/test_image.Dockerfile -t carsim-test:0.9.16 .

FROM --platform=linux/amd64 ubuntu:22.04 AS fetch
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates && rm -rf /var/lib/apt/lists/*
RUN git init -q /c && cd /c && git remote add origin https://github.com/carla-simulator/carla.git \
    && git config core.sparseCheckout true && echo PythonAPI/carla/agents/ > .git/info/sparse-checkout \
    && git fetch -q --depth 1 origin tag 0.9.16 && git checkout -q 0.9.16

FROM --platform=linux/amd64 ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive SDL_VIDEODRIVER=dummy
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-pip git ca-certificates \
    && rm -rf /var/lib/apt/lists/* && useradd -m carla && mkdir -p /work && chown carla /work
COPY --from=fetch --chown=carla /c/PythonAPI/carla/agents /work/stock/PythonAPI/carla/agents
COPY --chown=carla . /work/car_simulator
USER carla
WORKDIR /work/car_simulator
RUN bash scripts/setup_env_linux.sh
