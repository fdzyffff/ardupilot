
mkdir firmare
rm -fr firmare/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0X7nora
./waf plane
mkdir firmare/0X7nora
cp build/0X7nora/bin/arduplane.apj firmare/0X7nora/
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7pilot
./waf plane
mkdir firmare/0X7pilot
cp build/0X7pilot/bin/arduplane.apj firmare/0X7pilot/
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAV-X7
./waf plane
mkdir firmare/CUAV-X7
cp build/CUAV-X7/bin/arduplane.apj firmare/CUAV-X7/
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
