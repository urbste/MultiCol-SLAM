echo "Configuring and building Thirdparty/DBoW2 ..."

cd ThirdParty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


cd ../../
echo "Configuring and building Thirdparty/g2o ..."

EIGEN3_INCLUDE_DIR=$(pwd)/Eigen
export EIGEN3_INCLUDE_DIR

cd g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

echo "Configuring and building Thirdparty/opengv ..."

cd ../../OpenGV

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building MultiCol-SLAM ..."
cd ../../../
echo $(pwd)
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j