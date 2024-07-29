@echo off

docker compose build %1
docker builder prune -f