current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0X7u1.4
./waf plane
# mkdir firmware/0X7u1.4
cp build/0X7u1.4/bin/arduplane.apj firmware/$current_datetime\_0X7u1.4_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7u1.4-sim
./waf plane
# mkdir firmware/0X7u1.4
cp build/0X7u1.4/bin/arduplane.apj firmware/$current_datetime\_0X7u1.4-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
