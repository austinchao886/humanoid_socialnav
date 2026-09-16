# Local experimental overlay. Verify the base image ID before building.
ARG SONIC_BASE=social-motion/sonic:196f27c-workspace-v3
FROM ${SONIC_BASE}
COPY vendor/GR00T-WholeBodyControl/gear_sonic_deploy/src /sonic/gear_sonic_deploy/src
COPY pyproject.toml /pipeline/pyproject.toml
COPY motion_pipeline /pipeline/motion_pipeline
COPY packages /pipeline/packages
COPY services /pipeline/services
RUN python3 -m pip install --no-deps --no-build-isolation /pipeline \
    && cmake --build /sonic/gear_sonic_deploy/build --target g1_deploy_onnx_ref -j2
LABEL social-motion.experimental="composition-not-qualified"
CMD ["sonic-supervisor"]
