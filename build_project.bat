@echo off
echo Setting up ESP-IDF environment...

rem 使用错误消息中的ESP-IDF路径
set IDF_PATH=D:\software\Espressif\frameworks\esp-idf-v5.4

rem 检查环境
if not exist %IDF_PATH% (
    echo ESP-IDF not found at %IDF_PATH%
    echo Please modify this script with correct path
    pause
    exit /b 1
)

rem 执行ESP-IDF环境设置
call %IDF_PATH%\export.bat

echo Building project...
idf.py build

if %ERRORLEVEL% EQU 0 (
    echo Build successful!
) else (
    echo Build failed with error %ERRORLEVEL%
)

pause 