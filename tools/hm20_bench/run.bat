@echo off
chcp 65001 >nul
python bench_all.py --points 22 --threads 4
pause
