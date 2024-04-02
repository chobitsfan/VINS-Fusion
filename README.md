# VINS-Fusion

This branch is modified to work with Ardupilot and Luxonis OAK-D cameras.

## 1. Prerequisites
### 1.1. **Ceres Solver**
Tested with 2.1
```
    sudo apt install libceres-dev
```

### 1.2. **OpenCV**
Tested with 4.6
```
    sudo apt install libopencv-dev
```

## 2. Build
```
    cd vins_estimator
    cmake .
    make -j2
```

## 3. Run
```
    ./vins_fusion oak_d.yaml
```

## 4. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
