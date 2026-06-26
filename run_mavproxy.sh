mkdir multi_sitl_test
cd multi_sitl_test
mkdir mavproxy
cd mavproxy

mavproxy.py --out 127.0.0.1:14550 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501