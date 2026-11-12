#!/usr/bin/env bash
set -u
printf 'OS\n'
cat /etc/os-release
printf '\nCPU\n'
lscpu
printf '\nMEMORY\n'
free -h
printf '\nDISK\n'
df -h .
printf '\nPYTHON\n'
python3 --version
printf '\nGPU (only needed for optional viewer)\n'
if command -v glxinfo >/dev/null; then glxinfo -B; else echo 'glxinfo unavailable; headless MuJoCo does not require it'; fi
