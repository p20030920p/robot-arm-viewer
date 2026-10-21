@echo off
setlocal
cd /d "%~dp0"

if exist "runtime\node.exe" (
  "runtime\node.exe" serve.mjs
  exit /b %errorlevel%
)

where node >nul 2>nul
if errorlevel 1 (
  echo Node.js 18+ is required.
  echo Install it from https://nodejs.org/ or use the Windows release package.
  pause
  exit /b 1
)

node serve.mjs
