echo "------------- Kill all arducopter ------------- "
killall arducopter
mkdir multi_sim

echo "------------- Run Copter1 -------------"
mkdir multi_sim/Copter1
cd multi_sim/Copter1
../../build/sitl/bin/arducopter --model + --uartA tcp:0 --serial1 udpclient:127.0.0.1:14555 --serial2 mcast:226.0.0.22:6666 --instance 0 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter1.parm  --home 39.9788959,116.3397217,353.0,353 >copter1.out&
cd ../../


echo "------------- Run Copter2 -------------"
mkdir multi_sim/Copter2
cd multi_sim/Copter2
../../build/sitl/bin/arducopter --model + --uartA tcp:0 --serial1 udpclient:127.0.0.1:14555 --serial2 mcast:226.0.0.22:6666 --instance 1 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter2.parm  --home 39.9788507,116.3398303,353.0,353 >copter2.out&
cd ../../


echo ""------------- Run Mavproxy "-------------"

mavproxy.py --out 127.0.0.1:14550 --master udp:127.0.0.1:14555