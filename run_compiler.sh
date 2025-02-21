current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzF400
./waf copter
# mkdir firmare/APzF400
cp build/APzF400/bin/arducopter.apj firmare/$current_datetime\_APzF400_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzH7
# ./waf heli
# # mkdir firmare/APzH7
# cp build/APzH7/bin/arducopter-heli.apj firmare/$current_datetime\_APzH7_arducopter-heli.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzF4
# ./waf copter
# # mkdir firmare/APzF4
# cp build/APzF4/bin/arducopter.apj firmare/$current_datetime\_APzF4_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzF4
# ./waf heli
# # mkdir firmare/APzF4
# cp build/APzF4/bin/arducopter-heli.apj firmare/$current_datetime\_APzF4_arducopter-heli.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"