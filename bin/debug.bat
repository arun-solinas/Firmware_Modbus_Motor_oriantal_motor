@echo off

if exist "%Bin_Path%\project.jdebug" (
	echo.
	echo Opening previous configuration in "%Bin_Path%\project.jdebug"
	echo.
	
	start /wait ozone -programfile -project "%Bin_Path%\project.jdebug"
) else (
	echo.
	echo Creating a new project.jdebug file.
	echo. Prerequisites: Download relevant drivers for microcontroller from  "https://www.keil.arm.com/devices/" 
	echo Extract it using zip to "%Bin_Path%\Keil.*.pack\"
	echo. 
	echo Set the following in the "New Project Wizard" box:
	echo 1. Select the svd file present in "%Bin_Path%\Keil.*.pack\CMSIS\SVD\"
	echo 2. Set the target interface as SWD, 4MHz, USB
	echo 3. Select the .elf file that has been compiled
	echo 4. Debug by pressing F5
	echo 5. While exiting, save the new project in "%Bin_Path%\project.jdebug"
	echo.

	start /wait ozone -programfile "%Build_Path%\%Project_Name%.elf"
)

