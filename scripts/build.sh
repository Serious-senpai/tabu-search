g++ --version

params="-O3 -Wall -shared -std=c++17 -fPIC $(python3-config --includes) -I extern/pybind11/include"
extension=$(python3-config --extension-suffix)

if [ "$1" == "debug" ]
then
    params="$params -D DEBUG"
    echo "Building in debug mode"
fi

g++ $params ts/utils/cpp_utils.cpp -o ts/utils/cpp_utils$extension
echo "Built ts/utils/cpp_utils$extension"

g++ $params ts/d2d/utils/cpp_utils.cpp -o ts/d2d/utils/cpp_utils$extension
echo "Built ts/d2d/utils/cpp_utils$extension"
