@echo off

set Project_Drive=%~d0
set Apps_Path=%Project_Drive%\apps

%Project_Drive%
cd %~dp0

for %%I in (.) do set Project_Name=%%~nxI
set Home_Path=%cd%
cd "%Home_Path%"

for /d %%d in (%Apps_Path%\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu*) do set Compiler_Path=%%d\tools\bin\
set path=%Compiler_Path%;%path%
set Bin_Path=%Home_Path%\bin
set path="%Bin_Path%";%path%
set Build_Path=%Home_Path%\Debug
set path="%Build_Path%";%path%

set path="%Apps_Path%\4coder";%path%
set path="%Apps_Path%\Git";%path%
set path="%Apps_Path%\Meld";%path%
set path="%Apps_Path%\make\bin";%path%
set path="%Apps_Path%\Jlink";%path%
set path="%Apps_Path%\Ozone";%path%
set path="%Apps_Path%\STM32CubeProgrammer\bin";%path%
set path="%Apps_Path%\System View";%path%

%windir%\system32\cmd.exe
