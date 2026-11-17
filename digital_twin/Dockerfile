# The build and run environment. Ubuntu 22.04 with Python 3.10: CARLA 0.9.16 documents 22.04, and its client wheels
# need Python 3.10+. The first package lines are the list from 0.9.16's build_linux.md (CARLA builds with Unreal's own
# clang, so no system clang). The rest are what Unreal's window, the scripts and the bundle need.
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
      build-essential g++-12 cmake ninja-build libvulkan1 python3 python3-dev python3-pip python3-venv autoconf \
      wget curl rsync unzip git git-lfs libpng-dev libtiff5-dev libjpeg-dev python-is-python3 \
      tzdata sed libtool libxml2-dev aria2 sudo xdg-user-dirs vulkan-tools libsdl2-2.0-0 libomp5 libnss3 \
      libgtk-3-0 libasound2 libxcursor1 libxi6 libxrandr2 libxss1 libxinerama1 libxcb-glx0 libdbus-1-3 zstd
# CARLA's pins for building its Python API with 3.10 (Util/Docker/requirements/3.10/build.txt), and the client itself
RUN pip3 install auditwheel==5.1.0 build==1.3.0 distro==1.9.0 packaging==24.0 pyelftools==0.31 setuptools==65.5.0 \
      wheel==0.43.0 carla==0.9.16
RUN useradd -m -u 1000 carla && echo 'carla ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/carla
