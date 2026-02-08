@echo off
echo ========================================
echo   pico-ice Servo PWM 编译脚本
echo ========================================
echo.

@echo off
call C:\Users\panh3\Downloads\oss-cad-suite\environment.bat

echo [1/4] general (Yosys)...
yosys -p "synth_ice40 -top Servo_Top -json servo.json" Servo_PWM.v angle_sweep.v Servo_Top.v UART.v UART_deco.v
if errorlevel 1 goto :error

echo.
echo [2/4] wiring (nextpnr)...
nextpnr-ice40 --up5k --package sg48 --json servo.json --pcf pico_ice.pcf --asc servo.asc
if errorlevel 1 goto :error

echo.
echo [3/4] generating bitstream (icepack)...
icepack servo.asc servo.bin
if errorlevel 1 goto :error

echo.
echo [4/4] converting to UF2...

python uf2conv.py servo.bin -o servo.uf2 -f 0x792e7263 -b 0
if errorlevel 1 goto :error

    echo.
    echo Attention: uf2conv is not installed, please run: pip install uf2-conv
    echo you can also manually flash servo.bin vua DFU
)

echo.
echo ========================================
echo   Compile finished!
echo   drag servo.uf2 to pico-ice U
echo ========================================
goto :end

:error
echo.
echo !! COMPILEERROR !!
pause
exit /b 1

:end
pause
