current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0X7nora
./waf plane
# mkdir firmare/0X7nora
cp build/0X7nora/bin/arduplane.apj firmare/$current_datetime\_0X7nora_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7nora-SIM
./waf plane
# mkdir firmare/0X7nora-SIM
cp build/0X7nora-SIM/bin/arduplane.apj firmare/$current_datetime\_0X7nora-SIM_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7pilot
./waf plane
# mkdir firmare/0X7pilot
cp build/0X7pilot/bin/arduplane.apj firmare/$current_datetime\_0X7pilot_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7pilot-SIM
./waf plane
# mkdir firmare/0X7pilot-SIM
cp build/0X7pilot-SIM/bin/arduplane.apj firmare/$current_datetime\_0X7pilot-SIM_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAV-X7
./waf plane
# mkdir firmare/CUAV-X7
cp build/CUAV-X7/bin/arduplane.apj firmare/$current_datetime\_CUAV-X7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAV-X7-SIM
./waf plane
# mkdir firmare/CUAV-X7-SIM
cp build/CUAV-X7-SIM/bin/arduplane.apj firmare/$current_datetime\_CUAV-X7-SIM_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
cp ArduPlane/param/* ./
