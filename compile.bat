@echo off
wsl.exe -d Ubuntu22.04-1 -e ./waf configure --board APzH7
wsl.exe -d Ubuntu22.04-1 -e ./waf copter
