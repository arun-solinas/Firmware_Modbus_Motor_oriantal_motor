@echo off

call build.bat

setlocal enabledelayedexpansion
set Uploader_Type=%1

echo.
if "!Uploader_Type!"=="" (
	echo Run upload.bat file with args: [1] JLink [2] STLink
	goto :eof
)

set "Ran_Uploader="

if !Uploader_Type!==1 (
	if exist "project.jflash" (
		echo Using configuration saved in "%Bin_Path%\project.jflash"
		echo.
		start /wait JFlash -open"%Build_Path%\%Project_Name%.elf,08000000" -openprj"%Bin_Path%\project.jflash" -hide -production -startapp -exit
		REM start /wait JFlash -open"%Build_Path%\%Project_Name%.elf,08000000" -openprj"%Bin_Path%\project.jflash" -hide -programverify -startapp -exit
	) else (
		echo Follow the steps below for configuring the JFlash initially which is done only once
		echo.
		echo 1. Update microcontroller details in "Options->Project Settings->MCU"
		echo 2. Connect to the JLink using "Target->Connect"
		echo 3. Upload code from "Target->Production Programming"
		echo.
		echo Note: The configuration is saved in "%Bin_Path%\project.jflash"
		echo This will be used next time to upload the code to the microcontroller
		echo.

		start /wait JFlash -open"%Build_Path%\%Project_Name%.elf,08000000" -saveprjas"%Bin_Path%\project.jflash"
	)	
	
	set Ran_Uploader=1
) 

if !Uploader_Type!==2 ( 
	STM32_Programmer_CLI -c port=SWD -w "%Build_Path%\%Project_Name%.elf 0x8000000" -rst
	set Ran_Uploader=1
) 

if ERRORLEVEL==1 (
	echo Upload Error. Check if  
	echo 1. Code is compiled properly
	echo 2. Cable is connected
	echo 3. Microcontroller is powered on
) else (
	echo Upload Success.
)

if !Ran_Uploader!==0 (
	echo Accepted args: [1] JLink [2] STLink
)

echo.