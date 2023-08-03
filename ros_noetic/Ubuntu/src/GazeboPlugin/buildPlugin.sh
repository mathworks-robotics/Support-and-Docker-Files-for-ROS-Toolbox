#/* Copyright 2019 The MathWorks, Inc. */
echo "Building Gazebo Co-Sim plugin ..."

########### build GazeboCoSim lib

mkdir build
cd build

cmake ..

echo "Compiling plugin ..."
make

