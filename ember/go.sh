#!/bin/zsh

reacTIVision > /dev/null 2> /dev/null&

udptunnel -s 3000 -v localhost/3333

kill %
