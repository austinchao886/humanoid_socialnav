# Reuse the already-installed pinned GMR/SMPL-X environment.
FROM social-motion/video-gem-gmr:16bebf4-bb1bbe4-workspace-v2
RUN pip install --no-cache-dir pyzmq==26.4.0
COPY motion_pipeline/pico_live.py motion_pipeline/pico_live_service.py /opt/motion_pipeline/motion_pipeline/
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ENTRYPOINT ["python", "-m", "motion_pipeline.pico_live_service"]
