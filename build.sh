mkdir -p build
cd build
cmake -B . -S ../
cmake --build . -j 15
