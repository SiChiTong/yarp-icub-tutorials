#!/bin/bash

# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Pasquale <giulia.pasquale@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

########## root directory
# set the directory where you should have all deep learning tutorials/assignments
LAB_DIR=/home/icub/vvv17_deep-learning
echo ${LAB_DIR}

########## iCW dataset directory
# pay attention to put a '/' at the end (see example below)
IMAGES_DIR=/home/icub/robot-code/datasets/iCW/
echo ${IMAGES_DIR}

########## CAFFE stuff

# path to caffe executable (you should have the Caffe_ROOT env variable in your ~/.bashrc)
CAFFE_BIN=${Caffe_ROOT}/build/tools/caffe
echo ${CAFFE_BIN}

# path to CaffeNet model (you should already have it if you followed instructions before arriving)
WEIGHTS_FILE=${Caffe_ROOT}/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel
echo ${WEIGHTS_FILE}

# path to executable computing mean image of a set of images
COMPUTE_MEAN_BIN=${Caffe_ROOT}/build/tools/compute_image_mean
echo ${COMPUTE_MEAN_BIN}

# path to executable creating the LMDB databases
CREATE_LMDB_BIN=${Caffe_ROOT}/build/tools/convert_imageset
echo ${CREATE_LMDB_BIN}

########## directory of this REPOSITORY and this EXERCISE
TUTORIAL_DIR=${LAB_DIR}/solution_dl-tuning
echo ${TUTORIAL_DIR}
EX=id_10objects_caffenet
echo ${EX}

########## SCRIPTS
# set the path to the scripts (that you have just built)

PARSE_LOG_SH=${TUTORIAL_DIR}/scripts/parse_caffe_log.sh
echo ${PARSE_LOG_SH}

PLOT_LOG_SH=${TUTORIAL_DIR}/scripts/plot_log.gnuplot
echo ${PLOT_LOG_SH}

CLASSIFY_IMAGE_LIST_BIN=${TUTORIAL_DIR}/scripts/src/build/classify_image_list_vvv17
echo ${CLASSIFY_IMAGE_LIST_BIN}

########## TRAIN, VALIDATION and TEST sets: list of images
FILELIST_TRAIN=${TUTORIAL_DIR}/${EX}/images_lists/train.txt
echo ${FILELIST_TRAIN}
FILELIST_VAL=${TUTORIAL_DIR}/${EX}/images_lists/val.txt
echo ${FILELIST_VAL}
FILELIST_TEST_DAY1=${TUTORIAL_DIR}/${EX}/images_lists/test_longer_day1.txt
echo ${FILELIST_TEST_DAY1}
FILELIST_TEST_DAY2=${TUTORIAL_DIR}/${EX}/images_lists/test_longer_day2.txt
echo ${FILELIST_TEST_DAY2}
LABELS_FILE=${TUTORIAL_DIR}/${EX}/images_lists/labels.txt
echo ${LABELS_FILE}

########## TRAIN (plus mean image), VALIDATION and TEST databases for caffe
LMDB_TRAIN=${TUTORIAL_DIR}/${EX}/lmdb_train/
echo ${LMDB_TRAIN}
BINARYPROTO_MEAN=${TUTORIAL_DIR}/${EX}/mean.binaryproto
echo ${BINARYPROTO_MEAN}
LMDB_VAL=${TUTORIAL_DIR}/${EX}/lmdb_val/
echo ${LMDB_VAL}

########## create DATABASES
rm -rf ${LMDB_TRAIN}
${CREATE_LMDB_BIN} --resize_width=256 --resize_height=256 --shuffle ${IMAGES_DIR} ${FILELIST_TRAIN} ${LMDB_TRAIN}
${COMPUTE_MEAN_BIN} ${LMDB_TRAIN} ${BINARYPROTO_MEAN}
rm -rf ${LMDB_VAL}
${CREATE_LMDB_BIN} --resize_width=256 --resize_height=256 --shuffle ${IMAGES_DIR} ${FILELIST_VAL} ${LMDB_VAL}
    
########## fine-tuning PROTOCOL(s)
# the names of the directories where you have your 
# train_val.prototxt, solver.prototxt and deploy.prototxt

declare -a PROTOCOL_LIST=("all-0" \
                          "conv-0_fc6N-2_fc7N-2" \
                          "conv-0_fc6-2_fc7-2" \
                          "conv-0_fc6-3_fc7-3" \
                          "conv-0_fc6N-4_fc7N-4" \
                          "all-3" \
                          "all-3_batch-8" \
                          "all-0_batch-1")

for PROTOCOL in "${PROTOCOL_LIST[@]}"
do
    echo ${PROTOCOL}
   
    ########## SOLVER --> ARCHITECTURE and TEST
    # path to the solver.prototxt, which points to the train_val.prototxt
    SOLVER_FILE=${TUTORIAL_DIR}/${EX}/${PROTOCOL}/solver.prototxt
    echo ${SOLVER_FILE}
    # path to deploy.prototxt
    DEPLOY_FILE=${TUTORIAL_DIR}/${EX}/${PROTOCOL}/deploy.prototxt
    echo ${DEPLOY_FILE}

    ########## TRAIN!

    cd ${TUTORIAL_DIR}/${EX}/${PROTOCOL}
    # cd to the folder is needed since in solver.prototxt 
    # we defined the relative path to the train_val.prototxt

    # try multiple times to account for stochasticity
    # for i in 1 2 3 4 5
    for i in 1
    do
    
        # call caffe executable for training
        ${CAFFE_BIN} train -solver ${SOLVER_FILE} -weights ${WEIGHTS_FILE} --log_dir=${TUTORIAL_DIR}/${EX}/${PROTOCOL}

        # parse the output to obtain readable tables
        #rm ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/caffeINFOtrain.txt
        #rm ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/caffeINFOval.txt
        # creates caffeINFOtrain.txt and caffeINFOval.txt
        ${PARSE_LOG_SH} ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/caffe.INFO

        # plot the parsed output using caffeINFOtrain.txt and caffeINFOval.txt
        # you can either use gnuplot (after sudo apt-get install gnuplot)
        gnuplot -e "iodir='${TUTORIAL_DIR}/${EX}/${PROTOCOL}'" ${PLOT_LOG_SH}
        # or matlab
        matlab -nodisplay -nodesktop -r "addpath('${TUTORIAL_DIR}/scripts'); try plot_log('${TUTORIAL_DIR}/${EX}/${PROTOCOL}'); catch; end; quit"

        # list all snapshots and take the last one
        # (you should have only this one, if you left snapshot_iter=0 in solver.prototxt
        snap_list=(`ls -t icw_iter*.caffemodel`)
        FINAL_SNAP=${TUTORIAL_DIR}/${EX}/${PROTOCOL}/${snap_list[0]}
        FINAL_MODEL=${TUTORIAL_DIR}/${EX}/${PROTOCOL}/final.caffemodel
        mv ${FINAL_SNAP} ${FINAL_MODEL}
        rm ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/icw_iter_*.solverstate
        rm ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/icw_iter_*.caffemodel

        ########## TEST!
        
        FINAL_MODEL=${TUTORIAL_DIR}/${EX}/${PROTOCOL}/final.caffemodel

        # choose whether you want to print and visualize the prediction for each tested image
        # the string can be either "true" or "false"
        PRINT_PREDICTIONS="true";
        # if the above is true, choose the rate [ms] of visualization 
        IMG_DELAY="50"

        # on day1
        ${CLASSIFY_IMAGE_LIST_BIN} ${DEPLOY_FILE} ${FINAL_MODEL} ${BINARYPROTO_MEAN} \
                           ${LABELS_FILE} ${IMAGES_DIR} ${FILELIST_TEST_DAY1} \
                           ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/test_acc_day1.txt ${PRINT_PREDICTIONS} ${IMG_DELAY}
        # on day2
        ${CLASSIFY_IMAGE_LIST_BIN} ${DEPLOY_FILE} ${FINAL_MODEL} ${BINARYPROTO_MEAN} \
                           ${LABELS_FILE} ${IMAGES_DIR} ${FILELIST_TEST_DAY2} \
                           ${TUTORIAL_DIR}/${EX}/${PROTOCOL}/test_acc_day2.txt ${PRINT_PREDICTIONS} ${IMG_DELAY}

        echo "********* Done ${PROTOCOL} - ${i}! *********"

    done
done


