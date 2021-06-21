cls
clang++ main.cpp -o camera_calibration.exe ^
  -I ../../3rd_party_libs/opencv3.4/include/ ^
  -L ../../3rd_party_libs/opencv3.4/lib_win_64/ ^
  -lopencv_world3414
