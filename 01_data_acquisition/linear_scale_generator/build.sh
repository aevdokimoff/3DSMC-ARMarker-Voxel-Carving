TIMEFORMAT=%3lU
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
pushd $SCRIPTPATH > /dev/null

echo "building..."
time clang++ main.cpp -O3 -o generate_scale || exit 1

echo "running..."
time ./generate_scale || exit 1

popd > /dev/null
