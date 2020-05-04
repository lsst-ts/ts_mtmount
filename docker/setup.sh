#!/usr/bin/env bash

source ~/miniconda3/bin/activate
source $OSPL_HOME/release.com
source /home/saluser/.bashrc

echo "# Starting MTMount CSC"
run_mtmount.py $RUN_ARG
echo "# CSC exited."