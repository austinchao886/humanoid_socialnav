ARG CUDA_VERSION=12.4.1
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-pip python3-venv git curl wget cmake clang build-essential libeigen3-dev libyaml-cpp-dev libzmq3-dev libmsgpack-dev nlohmann-json3-dev libgtest-dev \
    "libnvinfer-dev=10.13.3.9-1+cuda12.9" \
    "libnvinfer-headers-dev=10.13.3.9-1+cuda12.9" \
    "libnvinfer-headers-plugin-dev=10.13.3.9-1+cuda12.9" \
    "libnvinfer10=10.13.3.9-1+cuda12.9" \
    "libnvinfer-plugin-dev=10.13.3.9-1+cuda12.9" \
    "libnvinfer-plugin10=10.13.3.9-1+cuda12.9" \
    "libnvonnxparsers-dev=10.13.3.9-1+cuda12.9" \
    "libnvonnxparsers10=10.13.3.9-1+cuda12.9" \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends zlib1g-dev && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && python3 -m pip install --no-cache-dir "onnxruntime-gpu==1.22.0"
RUN wget -q https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-gpu-1.22.0.tgz -O /tmp/onnxruntime.tgz \
    && mkdir -p /opt/onnxruntime \
    && tar -xzf /tmp/onnxruntime.tgz --strip-components=1 -C /opt/onnxruntime \
    && rm /tmp/onnxruntime.tgz
COPY vendor/GR00T-WholeBodyControl /sonic
COPY pyproject.toml /pipeline/pyproject.toml
COPY motion_pipeline /pipeline/motion_pipeline
RUN python3 -m pip install "cyclonedds==0.10.2" /pipeline
ENV PYTHONPATH=/unitree_sdk2_python SONIC_ROOT=/sonic/gear_sonic_deploy
ENV TensorRT_ROOT=/usr onnxruntime_ROOT=/opt/onnxruntime
WORKDIR /sonic/gear_sonic_deploy
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j4
CMD ["sonic-supervisor"]
