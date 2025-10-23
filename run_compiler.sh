current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf copter
# mkdir firmare/APzH7
cp build/APzH7/bin/arducopter.apj firmare/$current_datetime\_APzH7_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf copter
# mkdir firmare/APzH7
cp build/APzH7/bin/arducopter.apj firmare/$current_datetime\_APzH7_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7-sim
./waf copter
# mkdir firmare/APzH7
cp build/APzH7-sim/bin/arducopter.apj firmare/$current_datetime\_APzH7_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf copter
# mkdir firmare/APzF4
cp build/APzF4/bin/arducopter.apj firmare/$current_datetime\_APzF4_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4-sim
./waf copter
# mkdir firmare/APzF4
cp build/APzF4-sim/bin/arducopter.apj firmare/$current_datetime\_APzF4_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board TH-2
./waf copter
# mkdir firmare/TH-2
cp build/TH-2/bin/arducopter.apj firmare/$current_datetime\_TH-2_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board TH-2-sim
./waf copter
# mkdir firmare/TH-2
cp build/TH-2-sim/bin/arducopter.apj firmare/$current_datetime\_TH-2_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"