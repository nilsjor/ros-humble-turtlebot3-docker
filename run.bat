@echo off

docker compose up -d husarnet-device husarnet-edge

if %ERRORLEVEL% NEQ 0 (
   pause
)
