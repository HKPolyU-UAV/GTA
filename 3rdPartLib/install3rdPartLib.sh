sudo apt-get install cmake libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake -y

cd yaml-cpp-0.6.2
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install
