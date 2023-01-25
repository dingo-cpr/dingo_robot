dingo_robot
============

Robot packages for Dingo

## Dingo Bringup Environment Variables
To facilitate customizability of the Dingo platform with sensors, we provide the following environment variables. 
> Note: these variables must be using in conjunction with those in the [Dingo desciption package](https://github.com/dingo-cpr/dingo/). 

### Base Dingo
```bash
export DINGO_MOTOR_PARAMS='base'
export DINGO_OMNI=0
export DINGO_MAG_CONFIG=$(catkin_find dingo_base config/mag_config_default.yaml --first-only)
```

### Accessories

#### 2D Laser
```bash
export DINGO_LASER=0
export DINGO_LASER_MODEL='lms1xx' # or 'ust10'
export DINGO_LASER_HOST='192.168.131.20'
export DINGO_LASER_MOUNT='front'
export DINGO_LASER_TOPIC='front/scan'
```

```bash
export DINGO_LASER_SECONDARY=0
export DINGO_LASER_SECONDARY_MODEL='lms1xx' # or 'ust10'
export DINGO_LASER_SECONDARY_HOST='192.168.131.20'
export DINGO_LASER_SECONDARY_MOUNT='rear'
export DINGO_LASER_SECONDARY_TOPIC='rear/scan'
```

#### 3D Laser
```bash
export DINGO_LASER_3D=0
export DINGO_LASER_3D_MODEL='vlp16' # or 'hdl32e'
export DINGO_LASER_3D_HOST='192.168.131.20'
export DINGO_LASER_3D_TOPIC='front/points'
```

#### Realsense
```bash
export DINGO_REALSENSE=0
export DINGO_REALSENSE_TOPIC='realsense'
export DINGO_REALSENSE_MOUNT='front'
```
#### Microstrain 
```bash
export DINGO_IMU_MICROSTRAIN=0
export DINGO_IMU_MICROSTRAIN_NAME='microstrain'
export DINGO_IMU_MICROSTRAIN_LINK='microstrain_link'
export DINGO_IMU_MICROSTRAIN_PORT='/dev/microstrain'
```

#### Wibotic
```bash
export DINGO_WIBOTIC_CHARGER=0
export DINGO_WIBOTIC_CHARGER_CAN='can0'
export DINGO_WIBOTIC_CHARGER_ID='10'
```