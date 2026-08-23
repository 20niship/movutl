FROM ubuntu:26.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl wget just cmake make ninja-build build-essential libpython3-dev python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

