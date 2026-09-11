cd /home/adcs-public-robot/Documents/socialnav_humanoid_ws/motion_pipeline

docker compose -f docker-compose.motion.yml \
  --profile gui \
  up -d isaac-visualizer