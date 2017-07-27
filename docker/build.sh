#!/usr/bin/env bash
group_id=$(cut -d: -f3 < <(getent group sudo))
#sudo docker build --no-cache=true\
sudo docker build \
  --build-arg user=$USER\
  --build-arg uid=$UID\
  --build-arg gid=$group_id\
  --build-arg home=$HOME\
  --build-arg shell=$SHELL\
  -f carnd-term2.Dockerfile -t carnd-term2 .
