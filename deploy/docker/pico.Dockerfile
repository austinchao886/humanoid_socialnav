# Publisher/diagnostics only; no DDS controller or robot device access.
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-dev python3-pip build-essential cmake ca-certificates && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir 'setuptools<81' wheel pybind11 'numpy==1.26.4' 'scipy==1.15.3' pyzmq msgpack msgpack-numpy joblib tqdm easydict loguru 'pin==2.7.0'
RUN python3 -m pip install --no-cache-dir 'torch==2.6.0+cpu' --index-url https://download.pytorch.org/whl/cpu
COPY vendor/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64 /opt/xrt
RUN CMAKE_PREFIX_PATH=$(python3 -m pybind11 --cmakedir) python3 -m pip install --no-build-isolation /opt/xrt
ENV LD_LIBRARY_PATH=/opt/xrt/lib PYTHONPATH=/sonic
WORKDIR /sonic
CMD ["python3", "gear_sonic/scripts/pico_manager_thread_server.py", "--help"]
