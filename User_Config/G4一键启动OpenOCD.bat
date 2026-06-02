@echo off
chcp 65001 > nul
title STM32G4 DAPLink 调试服务器启动器

REM ===================== 【配置区】请修改为你自己的绝对路径 =====================
REM 1. OpenOCD 根目录（绝对路径，到文件夹为止，不要到 bin）
set "OPENOCD_ROOT=C:\DevEnv\DevEnv\OpenOCD"

REM 2. DAPLink 配置文件（绝对路径，到 .cfg 文件）
set "DAPLINK_CFG=D:\KnowledgeBase\MyProject\BSP\H7_BSP\User_Config\ozone_daplink.cfg"

REM 3. 等待 OpenOCD 启动的时间（秒，一般不用改）
set "WAIT_TIME=2"
REM ==============================================================================

REM 自动拼接 OpenOCD 相关路径（无需修改）
set "OPENOCD_EXE=%OPENOCD_ROOT%\bin\openocd.exe"
set "TARGET_CFG=%OPENOCD_ROOT%\share\openocd\scripts\target\stm32g4x.cfg"

REM 检查文件是否存在，避免路径错误
if not exist "%OPENOCD_EXE%" (
    echo [错误] 未找到 OpenOCD：%OPENOCD_EXE%
    echo 请检查配置区的 OPENOCD_ROOT 路径是否正确！
    pause
    exit /b 1
)
if not exist "%DAPLINK_CFG%" (
    echo [错误] 未找到 DAPLink 配置文件：%DAPLINK_CFG%
    echo 请检查配置区的 DAPLINK_CFG 路径是否正确！
    pause
    exit /b 1
)

REM 1. 启动 OpenOCD 调试服务器
echo 正在启动 OpenOCD 调试服务器...
echo.
echo 连接信息：
echo - GDB 连接端口：3333
echo - Telnet 连接端口：4444
echo - TCL 连接端口：6666
echo.
echo 提示：不要关闭此窗口，否则调试会断开！
echo.

start "OpenOCD - STM32G4 调试服务器" cmd /k ""%OPENOCD_EXE%" -f "%DAPLINK_CFG%" -f "%TARGET_CFG%""

REM 2. 等待服务器启动
timeout /t %WAIT_TIME% /nobreak > nul

echo OpenOCD 调试服务器已启动！
echo 现在可以用 Ozone 或其他工具连接 localhost:3333 进行调试。
echo.
timeout /t 2 > nul
exit /b 0
