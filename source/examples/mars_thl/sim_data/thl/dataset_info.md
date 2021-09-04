## IMU
```
n_w = [0.013; 0.013; 0.013]; % Noise STD [rad/s]
n_bw = [0.0013; 0.0013; 0.0013];
n_a = [0.083; 0.083; 0.083]; % Noise STD [m/s^2]
n_ba = [0.0083; 0.0083; 0.0083];
```

## GPS position
```
Calibration
p_ig=[0.0, 0.0, 0.0]

Noise (STD)
R=[0.8545, 0.8545, 2.126]
```

### GPS velocity

```
Calibration
p_ig=[0.0 0.0 0.0]

Noise (STD)
R_p=[0.8545, 0.8545, 2.126]
R_v=[0.01, 0.01, 0.01]
```


### Pressure Sensor 1

```
Calibration
p_ip = [0.15, 0.0, 0.0];

Noise (STD)
R=[0.15]
```



### Pressure Sensor 2

```
Calibration
p_ip = [0.0, 0.8, 0.0];

Noise (STD)
R=[0.15]
```


### Magnetometer

```
Calibration
q_im = Quaternion([1 0 0 0]);

variation_world_s.az = 75.5*pi/180;
variation_world_s.el = 55.5*pi/180;
variation_world_s.r = mag.strength;

Noise (STD)
R_spher = [0.012654532165944 0.007246709209877 19.823780830776336]
```

### Pose sensor 1

```
Calibration
p_ip = [0.0, 0.0, 0.0];
q_ip = Quaternion(rpy2r([deg2rad(0) deg2rad(0) deg2rad(0)]))

Noise (STD)
R_p = [0.02 0.02 0.02]
R_r = [deg2rad(2), deg2rad(2), deg2rad(2)]
```

### Pose sensor 2

```
Calibration
p_ip = [0.10 0.10 0.3];
q_ip = Quaternion(rpy2r([deg2rad(0) deg2rad(0) deg2rad(45)]))

Noise (STD)
R_p = [1.0 1.0 1.0]
R_r = [deg2rad(10), deg2rad(10), deg2rad(10)]
```

### Pose sensor 3

```
Calibration
p_ip = [2.0 1.0 0.5];
q_ip = Quaternion(rpy2r([deg2rad(30) deg2rad(30) deg2rad(45)]))

Noise (STD)
R_p = [5.0 5.0 10.0]
R_r = [deg2rad(30), deg2rad(30), deg2rad(30)]
```

### Pose sensor 4

```
Calibration
p_ip = [0.20 0.20 0.0];
q_ip = Quaternion(rpy2r([deg2rad(45) deg2rad(0) deg2rad(45)]))

Noise (STD)
R_p = [0.05 0.05 0.05]
R_r = [deg2rad(1), deg2rad(1), deg2rad(1)]
```
