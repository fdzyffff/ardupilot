#in console 1st, recommand to use sim_vehicle.py for a terminal to control

cd /mnt/d/git_code/ArduPilot/Plane-4.3.1-movablelanding/ardupilot/ArduPlane/
../Tools/autotest/sim_vehicle.py -f quadplane-quad -A "--uartC=uart:/dev/ttyS60:115200"

OR

/mnt/d/git_code/ArduPilot/Plane-4.3.1-movablelanding/ardupilot/build/sitl/bin/arduplane -S --model quadplane-quad --speedup 1 --slave 0 --uartC=uart:/dev/ttyS60:115200 --defaults ../Tools/autotest/default_params/quadplane.parm -I0


# in console 2nd, recommand to use the direct model cmd for avoiding bug of mavproxy

cd /mnt/d/git_code/ArduPilot/Plane-4.3.1-movablelanding/ardupilot/Rover/
../Tools/autotest/sim_vehicle.py -A "--uartC=uart:/dev/ttyS61:115200" -I 2

OR

/mnt/d/git_code/ArduPilot/Plane-4.3.1-movablelanding/ardupilot/build/sitl/bin/ardurover -S --model rover --speedup 1 --slave 0 --uart0=tcp:5780 --uartC=uart:/dev/ttyS61:115200 --defaults ../Tools/autotest/default_params/rover.parm -I2