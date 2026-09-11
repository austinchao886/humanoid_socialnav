FROM nvcr.io/nvidia/pytorch:24.10-py3

ENV DEBIAN_FRONTEND=noninteractive PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONUNBUFFERED=1
WORKDIR /workspace/kimodo
RUN apt-get update && apt-get install -y --no-install-recommends git curl ca-certificates cmake build-essential ffmpeg gosu && rm -rf /var/lib/apt/lists/*
COPY vendor/kimodo/docker_requirements.txt ./docker_requirements.txt
COPY vendor/kimodo/setup.py vendor/kimodo/pyproject.toml ./
COPY vendor/kimodo/kimodo ./kimodo
COPY vendor/kimodo/kimodo-viser ./kimodo-viser
COPY vendor/kimodo/MotionCorrection ./MotionCorrection
# The NGC base image adds pypi.ngc.nvidia.com as an extra index. It is not
# needed for these pinned packages and makes every lookup wait on DNS fallback.
ENV PIP_CONFIG_FILE=/dev/null
RUN rm -f /usr/local/bin/cmake && SKIP_MOTION_CORRECTION_IN_SETUP=1 python -m pip install -r docker_requirements.txt
COPY pyproject.toml /pipeline/pyproject.toml
COPY motion_pipeline /pipeline/motion_pipeline
COPY packages /pipeline/packages
COPY services /pipeline/services
RUN python -m pip install "cyclonedds==0.10.2" /pipeline
RUN python -m pip install "mujoco==3.3.7"
ENV PYTHONPATH=/unitree_sdk2_python
ARG PIPELINE_COMMIT=unknown
LABEL org.opencontainers.image.revision=$PIPELINE_COMMIT
CMD ["motion-service"]
