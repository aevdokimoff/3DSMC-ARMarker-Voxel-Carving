SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
pushd $SCRIPTPATH > /dev/null

cmake -S . -B build
cmake --build build || exit 0


./build/image_realign_advisor "./01_data_acquisition/images/obj_unicorn/"
./build/image_realign_advisor "./01_data_acquisition/images/obj_duck/"
./build/image_realign_advisor "./01_data_acquisition/images/obj_owl/"

pushd build  > /dev/null
./marching_cubes owl      || exit 1
./marching_cubes unicorn  || exit 1
./marching_cubes duck     || exit 1

# ./texture_backprojection                    \
#     "../01_data_acquisition/images/obj_owl" \
#     "results/owl_processed.obj" "results/owl_processed.png" 2048 || exit 1

# ./texture_backprojection                        \
#     "../01_data_acquisition/images/obj_unicorn" \
#     "results/unicorn_processed.obj" "results/unicorn_processed.png" 2048 || exit 1

# ./texture_backprojection                     \
#     "../01_data_acquisition/images/obj_duck" \
#     "results/duck_processed.obj" "results/duck_processed.png" 2048 || exit 1

popd > /dev/null # build
popd > /dev/null # scriptpath
