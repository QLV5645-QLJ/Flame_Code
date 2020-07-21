objdump -f liblxrt.so | grep ^architecture

# install needed lib
apt-get update
apt-get install build-essential
apt-get install libncurses5-dev
apt-get install zip
apt-get install openssh-client


download ud-driver: http://www.diamondsystems.com/products/uddown.php?key=Z898W4SQ

# install 2.9
http://fengnz.blogspot.com/2018/05/compile-gcc-2953-with-docker-gcc.html

##add following line to /etc/apt/source.list
deb http://archive.debian.org/debian etch main

# install 3.4
https://askubuntu.com/questions/923337/installing-an-older-gcc-version3-4-3-on-ubuntu-14-04-currently-4-8-installed



# install gcc and g++ 2.95 
apt-get update && apt-get install gcc-2.95 g++-2.95 -y
mv /usr/bin/gcc-2.95 /usr/bin/gcc
mv /usr/bin/g++-2.95 /usr/bin/g++
apt-get install -y make
apt-get install -y libncurses5-dev
apt-get install -y zip unzip
apt-get install -y openssh-client

docker run -it flame_os --network="host"

# try
https://opensource.com/article/19/7/cross-compiling-gcc -march=i686

cc -o calibrate_DMM16AT calibrate_DMM16AT.c -g3 -O2 -I.. -I/usr/local/lib/dscud5 -L/usr/local/lib/dscud5 -ldscud5 -lpthread -lm
calibrate_DMM16AT.c:15:10: fatal error: dscud.h: No such file or directory
 #include <dscud.h>
