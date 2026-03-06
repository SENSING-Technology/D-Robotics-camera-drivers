#!/bin/sh
set -e
local_path=$(dirname "$(readlink -f "$0")")
cd "${local_path}"/sunrise_camera/bin || exit 1

echo "============= Start Sunrise Camera ==============="
if [ "$#" -eq 0 ]; then
	./sunrise_camera
else
	gdb -ex "handle SIGUSR2 nostop" -ex "handle SIGPIPE nostop" -ex "run"  ./sunrise_camera
fi
