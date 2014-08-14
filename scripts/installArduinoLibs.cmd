@echo off
For /f "tokens=2-4 delims=/ " %%a in ('date /t') do (set mydate=%%c-%%a-%%b)
For /f "tokens=1-2 delims=/:" %%a in ('time /t') do (set mytime=%%a%%b)
echo %mydate%_%mytime%


set ARDUINO_FOLDER=%USERPROFILE%\Documents\Arduino
set ARDUINO_LIB_FOLDER=%ARDUINO_FOLDER%\libraries
set ARDUINO_LIB_BACKUP_FOLDER="%ARDUINO_LIB_FOLDER%_backup_%mydate%_%mytime%"

move %ARDUINO_LIB_FOLDER% %ARDUINO_LIB_BACKUP_FOLDER% 

xcopy avr\libraries %ARDUINO_LIB_FOLDER%\   /s /e /h /y /q