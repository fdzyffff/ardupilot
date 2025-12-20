current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf plane
# mkdir firmare/APzH7
cp build/APzH7/bin/arduplane.apj firmare/$current_datetime\_APzH7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7-sim
./waf plane
# mkdir firmare/APzH7
cp build/APzH7-sim/bin/arduplane.apj firmare/$current_datetime\_APzH7_sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf plane
# mkdir firmare/APzF4
cp build/APzF4/bin/arduplane.apj firmare/$current_datetime\_APzF4_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4-sim
./waf plane
# mkdir firmare/APzF4
cp build/APzF4-sim/bin/arduplane.apj firmare/$current_datetime\_APzF4_sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"