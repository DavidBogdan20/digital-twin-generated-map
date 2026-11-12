# The build environment. Ubuntu 20.04 is the newest system CARLA 0.9.15 documents; Google's driver needs 22.04+ on the host.
# The first package lines are the list from 0.9.15's build_linux.md. The rest are what Unreal's window needs.
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
      build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python-dev python3-dev \
      python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync \
      libxml2-dev git \
      aria2 wget sudo xdg-user-dirs vulkan-utils libsdl2-2.0-0 libomp5 libnss3 libgtk-3-0 libasound2 \
      libxcursor1 libxi6 libxrandr2 libxss1 libxinerama1 libxcb-glx0 libdbus-1-3
RUN pip3 install -Iv setuptools==47.3.1 && pip3 install distro wheel auditwheel
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180
RUN useradd -m -u 1000 carla && echo 'carla ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/carla
