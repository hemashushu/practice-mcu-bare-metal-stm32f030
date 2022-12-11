target extended-remote localhost:3333
set backtrace limit 32
source svd/gdb.py
svd_load svd/STM32F0x0.svd
load
