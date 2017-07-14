#!/usr/bin/env bash

sudo nvidia-docker run\
  --net=host\
  -e SHELL\
  -e DISPLAY=$DISPLAY\
  -e DOCKER=1\
  -v `pwd`:/src\
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 4567:4567\
  -p 22:22\
  -it --rm carnd-term2 /bin/bash

  #-v `pwd`:/src
  #-v "$HOME:$HOME:rw"\
