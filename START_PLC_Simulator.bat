@echo off
chcp 65001 > nul
echo ========================================
echo   PLC Simulator 시작
echo   Modbus TCP Server: 0.0.0.0:502
echo ========================================
echo.
echo [주의] 이 프로그램은 관리자 권한이 필요합니다 (포트 502)
echo.
cd /d "%~dp0"

REM Python 가상환경 확인 및 생성
if not exist venv (
    echo [설치] Python 가상환경 생성 중...
    python -m venv venv
    if errorlevel 1 (
        echo ERROR: Python 가상환경 생성 실패
        echo INFO: Python 3.8 이상이 설치되어 있는지 확인하세요
        pause
        exit /b 1
    )

    echo [설치] 의존성 패키지 설치 중...
    venv\Scripts\pip install -r requirements.txt
    if errorlevel 1 (
        echo ERROR: 패키지 설치 실패
        pause
        exit /b 1
    )
)

echo.
echo ========================================
echo   PLC Simulator Starting...
echo ========================================
echo.

venv\Scripts\python.exe -u plc_simulator.py

echo.
echo ========================================
echo   PLC Simulator 종료
echo ========================================
pause
