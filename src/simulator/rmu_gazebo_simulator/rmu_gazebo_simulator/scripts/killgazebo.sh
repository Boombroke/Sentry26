#!/bin/bash
ps -ef | grep 'gz sim server' | cut -c 9-15 | xargs kill -s 15
