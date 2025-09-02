#!/bin/bash
<<COMMENT
#
# 2025-08-30 RDK S100 V4.0.2
#
COMMENT

clear

red_print(){
    echo -e "\e[1;31m$1\e[0m"
}
green_print(){
    echo -e "\e[1;32m$1\e[0m"
}

red_print "This package is use for RDK S100 on V4.0.2"

echo 1:SG1S-OX01F10C-G1G
echo 2:SG2-AR0231C-0202-GMSL
echo 3:SG2-AR0233C-5200-G2A
echo 4:SG3S-ISX031C-GMSL2F
echo 5:SHW3H
echo 6:SHF3L
echo 7:SG8S-AR0820C-5300-G2A

green_print "Press select your camera type:"
read key

if [ ${key} -eq 1 -o ${key} -eq 2 ];then
	sudo dpkg -i ./so/hobot-camera_4.0.2-20250830134613_arm64_gmsl1.deb
else
	sudo dpkg -i ./so/hobot-camera_4.0.2-20250829164010_arm64.deb
fi

