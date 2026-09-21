# Preserve the deployed composition supervisor; apply only the guarded PICO hook.
FROM social-motion/sonic:composition-candidate-20260915-08
COPY motion_pipeline/pico_live.py /usr/local/lib/python3.10/dist-packages/motion_pipeline/pico_live.py
COPY services/sonic_tracker/sonic_tracker/pico_replay.py services/sonic_tracker/sonic_tracker/pico_live_session.py /usr/local/lib/python3.10/dist-packages/sonic_tracker/
COPY deploy/docker/install_pico_live_hook.py /tmp/install_pico_live_hook.py
RUN python3 /tmp/install_pico_live_hook.py
ENV SONIC_ENABLE_PICO_LIVE=0
