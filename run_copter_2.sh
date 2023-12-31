cd copter_2
/mnt/d/git_code/ArduPilot/Copter-4.4.3-zhongfei/ardupilot/build/sitl/bin/arducopter -S --model + --speedup 1 --slave 0 --instance 2 --defaults ../Tools/autotest/default_params/copter.parm,./2.parm --sim-address=127.0.0.1 -I0 --uartA tcp:0 --uartC tcpclient:127.0.0.1:5777 --uartD udpclient:127.0.0.1:14550
cd ../