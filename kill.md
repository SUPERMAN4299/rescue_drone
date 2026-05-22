taskkill /F /IM python.exe

for /d /r . %d in (__pycache__) do @if exist "%d" rd /s /q "%d" - cmd
Get-ChildItem -Path . -Filter "__pycache__" -Recurse -Directory | Remove-Item -Force -Recurse -- powershell

C:\Users\HP\AppData\Roaming\Ultralytics - delete cache

netstat -ano | findstr :5000
netstat -ano | findstr :3001
taskkill /PID PID_NUMBER /F

Ctrl + Shift + Delete - brower

restarts