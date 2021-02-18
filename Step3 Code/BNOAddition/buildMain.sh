#/bin/sh

WALABOT_SHARED_LIB_PATH=/usr/lib/walabot

g++ -o main.out main.cpp -O2 -D__LINUX__ -lWalabotAPI -Wl,-rpath,$WALABOT_SHARED_LIB_PATH -L$WALABOT_SHARED_LIB_PATH -lsfml-audio -lpthread -lpigpio -lstdc++
