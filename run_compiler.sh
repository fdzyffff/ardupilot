current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0X7u1.2
./waf plane
# mkdir firmware/0X7u1.2
cp build/0X7u1.2/bin/arduplane.apj firmware/$current_datetime\_0X7u1.2_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
