#!/bin/sh

# This script puts the Cuda option of CMakeLists.txt in ON or OFF

change_param() {
	copt=$(grep -n set\(CUDA_ENABLE CMakeLists.txt | awk '{print $NF}')
	uopt=$(echo $1 | tr [a-z] [A-Z])
	sed -i "s/set(CUDA_ENABLE $copt/set(CUDA_ENABLE $uopt\)/g" CMakeLists.txt 2>/dev/null
}


usage() {
	echo Usage: $0 \(on/off\) 1>&2
	exit 1
}

if [ $# -ne 1 ]
then
	usage
fi

test $1 = "on" && change_param $1
test $1 = "off" && change_param $1

exit 0
