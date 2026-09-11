FROM pytorch/pytorch:2.6.0-cuda12.4-cudnn9-runtime

ARG PIPELINE_COMMIT=unknown
LABEL org.opencontainers.image.revision=$PIPELINE_COMMIT

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1 GEM_ROOT=/opt/GEM GMR_ROOT=/opt/GMR \
    PYOPENGL_PLATFORM=egl MUJOCO_GL=egl

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ffmpeg git libegl1 libgl1 libglib2.0-0 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/NVlabs/GENMO.git /opt/GEM \
    && git -C /opt/GEM checkout 16bebf402d8893184249ee206d957b8248cd8310 \
    && git clone https://github.com/YanjieZe/GMR.git /opt/GMR \
    && git -C /opt/GMR checkout bb1bbe40774794fceb2a7c579a3464a28e68c844

COPY . /opt/motion_pipeline

# GEM pins NumPy 1.23.5. Do not let the pipeline's generic dependency
# declaration upgrade the model environment to an incompatible NumPy.
RUN pip install --no-cache-dir -e /opt/GEM \
    && pip install --no-cache-dir -e /opt/GMR \
    && pip install --no-cache-dir --no-deps -e /opt/motion_pipeline

# 0.10.2 has no CPython 3.11 wheel; 11.0.1 retains the String_ IDL/channel API
# used here and ships an audited manylinux wheel with its native runtime.
RUN pip install --no-cache-dir "cyclonedds==11.0.1"

WORKDIR /opt/GEM
CMD ["video-motion-service"]
