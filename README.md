# SLAM_C_plus_plus

## EKF-SLAM Example

This project contains a conceptual implementation of EKF-SLAM using Eigen for matrix operations.

### File:
- `slam_ekf_eigen.cpp`

### Description:
A conceptual EKF-SLAM demonstration using Eigen for matrix operations.

### Requirements:
Make sure you have Eigen installed:
```bash
sudo apt-get install libeigen3-dev  # Ubuntu/Debian
```

Alternatively, this project includes Eigen as a Git submodule.

### Compilation:
```bash
g++ -I./eigen -std=c++14 slam_ekf_eigen.cpp -o ekf_slam
```

## Cloning This Repository
This project includes the Eigen library as a submodule.

To clone with Eigen:
```bash
git clone --recurse-submodules https://github.com/AliYavari2000/SLAM_C_plus_plus.git
```

If already cloned:
```bash
git submodule update --init --recursive
```
