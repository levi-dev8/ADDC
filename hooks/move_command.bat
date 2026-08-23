@echo off
rem hooks/move_command.bat <direction> <magnitude> <reason>
set DIRECTION=%~1
set MAGNITUDE=%~2
set REASON=%~3

echo [MoveCmd] %DATE% %TIME% - Direction: %DIRECTION% ^| Magnitude: %MAGNITUDE% ^| Reason: %REASON%
