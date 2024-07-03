@echo off

docker compose up -d talker listener

if %ERRORLEVEL% NEQ 0 (
   pause
)
