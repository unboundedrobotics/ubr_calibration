#Calibration Sandbox

This package is a prototype of a potential calibration solution. It is still very much under development.

The package current consists of two nodes:
 * capture - this node moves the arm and head around, sees the led on the gripper and 
   creates CalibrationData messages that can be used by:
 * calibration - this node actually does the optimization and outputs a new URDF.

##Major Dependencies
 * Ceres (BSD License)
 * KDL (LGPL License)

##Installing Ceres

    # install google gflags
    # In quantal and above, this is libgoogle-glog-devlibflags-dev
    wget http://gflags.googlecode.com/files/gflags-2.0.tar.gz
    tar -xvzf gflags-2.0.tar.gz
    cd gflags-2.0
    ./configure --prefix=/usr/local
    make
    sudo make install

    # install google glog
    # In quantal and above, this is libgoogle-glog-dev
    wget http://google-glog.googlecode.com/files/glog-0.3.3.tar.gz
    tar -xvzf glog-0.3.3.tar.gz
    cd glog-0.3.3
    ./configure --with-gflags=/usr/local/
    make -j8
    sudo make install

    # system depends
    sudo apt-get install libsuitesparse-dev libprotobuf-dev

    # ceres
    wget http://ceres-solver.googlecode.com/files/ceres-solver-1.6.0.tar.gz
    tar zxf ceres-solver-1.6.0.tar.gz
    mkdir ceres-bin
    cd ceres-bin
    cmake ../ceres-solver-1.6.0
    make -j8
    sudo make install
