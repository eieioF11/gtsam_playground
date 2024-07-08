# gtsam_playground

# icp_test

## Install Boost
```bash
sudo apt-get install libboost-all-dev
```

## Install gtsam
https://gtsam.org/get_started/
```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## build
```bash
cd build
cmake ..
make
```

