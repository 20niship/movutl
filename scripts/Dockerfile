FROM ubuntu:26.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl wget just cmake make ninja-build build-essential libpython3-dev python3-venv python3-pip \
    default-jdk npm nodejs git cmake make wget lcov curl libfmt-dev libssl-dev \
    g++-14 gcc-14 \
    mesa-vulkan-drivers libopencv-dev \
    libglu1-mesa-dev libglfw3-dev libglew-dev libglm-dev ninja-build \
    libfreetype-dev libeigen3-dev libassimp-dev libpcl-dev libbullet-dev libopenal-dev \
    python3.12-venv libpython3.12-dev python3.12 \ 
    libalut-dev libogg-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavdevice-dev libffmpeg-nvenc-dev  \
    xfonts-scalable libocct-data-exchange-dev libocct-draw-dev libocct-foundation-dev libocct-modeling-algorithms-dev \
    libocct-modeling-data-dev libocct-ocaf-dev libocct-visualization-dev  libxrandr-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev && \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

