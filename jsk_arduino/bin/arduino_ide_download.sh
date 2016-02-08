#!/bin/sh

wget -c http://arduino.googlecode.com/files/arduino-1.0.5-linux64.tgz
cd ../ && tar -zxvf  bin/arduino-1.0.5-linux64.tgz
rm bin/*.tgz
