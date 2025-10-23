current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0x7u1.4
./waf copter
# mkdir firmare/0x7u1.4
cp build/0x7u1.4/bin/arducopter.apj firmare/$current_datetime\_0x7u1.4_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0x7u1.4-sim
./waf copter
# mkdir firmare/0x7u1.4
cp build/0x7u1.4-sim/bin/arducopter.apj firmare/$current_datetime\_0x7u1.4-sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"