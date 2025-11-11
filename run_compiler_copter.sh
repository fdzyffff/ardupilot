current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
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