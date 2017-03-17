#!/bin/bash

# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# Put here those instructions we need to execute before running the test


# case "$OSTYPE" in
#   darwin*)  alias matlab='/Applications/MATLAB_R2016b.app/bin/matlab' ;;
# esac
#
export YARP_ROBOT_NAME=icubGazeboSim

OLDMATLABPATH=${MATLABPATH}
export MATLABPATH=${MATLABPATH}:`pwd`
