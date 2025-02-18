current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf copter
mkdir firmare/APzH7
cp build/APzH7/bin/arducopter.apj firmare/APzH7/$current_datetime\_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf heli
mkdir firmare/APzH7
cp build/APzH7/bin/arducopter-heli.apj firmare/APzH7/$current_datetime\arducopter-heli.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf copter
mkdir firmare/APzF4
cp build/APzF4/bin/arducopter.apj firmare/APzF4/$current_datetime\arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf heli
mkdir firmare/APzF4
cp build/APzF4/bin/arducopter-heli.apj firmare/APzF4/$current_datetime\arducopter-heli.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"