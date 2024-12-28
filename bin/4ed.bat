@echo off
pushd "%Bin_Path%"
set Main_File=%Home_Path%/Core/Src/main.cpp
set Default_Files="../.gitignore" "%Apps_Path%/4coder/bindings.4coder" "%Home_Path%/cmd.bat" "%Apps_Path%/4coder/config.4coder" 
if exist  %Main_File% (
	start 4coder.exe %Main_File% %Default_Files% -f 16
) else (
	start 4coder.exe %Default_Files% -f 16
)
popd