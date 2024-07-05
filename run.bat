@echo off

docker compose up -d talker listener discovery

if %ERRORLEVEL% NEQ 0 (
   pause
)
