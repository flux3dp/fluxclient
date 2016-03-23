#!/usr/bin/env bash

# This is a shell script running unit test on jenkins
# One should not run this manually (run "setup.py test" or "py.test")
set -x

SLIC3R=$2
CLIENT_BRANCH=$1

TODAY=$(date +%Y%m%d)

export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin

source ~/py35/bin/activate
if [ "" != "$SLIC3R" ]; then
    export slic3r=$SLIC3R
    python -c "import os;print(os.environ['slic3r'])"
fi


if [ "" == "$CLIENT_BRANCH" ]; then
    CLIENT_BRANCH="master"
fi

# update fluxclient

source ~/py35/bin/activate

cd ~/py35/fluxclient
git fetch --all
git checkout $CLIENT_BRANCH
git reset --hard origin/$CLIENT_BRANCH
git clean -df
git pull
./setup.py test || exit 1
