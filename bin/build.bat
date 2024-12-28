@echo off
REM echo D:\code\cube-ide\endobot_firmware\Core\Inc\endobot.h:1023: warning: unused variable 'status' [-Wunused-variable]

pushd %Home_Path%\Debug
make -j16 all
popd