@echo off
pushd %~dp0

echo.
echo compiling...
pushd bin
cl ..\main.cpp ..\..\common\image.cpp /O2 /std:c++latest /nologo /openmp || exit /b 1

echo.
echo running...
main.exe

popd
popd
