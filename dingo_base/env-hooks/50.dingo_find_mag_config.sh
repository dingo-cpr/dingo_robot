DINGO_MAG_CONFIG=$(catkin_find --etc --first-only dingo_base mag_config.yaml 2>/dev/null)
if [ -z "$DINGO_MAG_CONFIG" ]; then
  DINGO_MAG_CONFIG=$(catkin_find --share --first-only dingo_base config/mag_config_default.yaml 2>/dev/null)
fi

export DINGO_MAG_CONFIG
