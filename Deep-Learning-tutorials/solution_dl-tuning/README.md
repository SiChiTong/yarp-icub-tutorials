# Fine-tuning deep CNNs with Caffe

## How to complete the assignment

We suppose that you already followed instructions provided to run the DL tutorial [here](https://github.com/vvv-school/tutorial_dl-tuning) and that your are all set.

#### Get the code

Navigate to the DL folder and clone this repository:

```sh
$ LAB_DIR=/home/icub/vvv17_deep-learning
$ cd $LAB_DIR
$ git clone https://www.github.com/vvv-school/assignment_dl-tuning.git
```

#### Compile the code

Compile the scripts that are provided with the repository:

```sh
$ cd $LAB_DIR
$ cd assignment_dl-tuning/scripts/src
$ mkdir build
$ cd build
$ ccmake ../
$ make
```

NOTES: 

1. you do not have to install this code, `make` is sufficient
2. check that `CPU_ONLY=ON` and `USE_CUDNN=OFF` if you have not built `caffe` for the GPU (which is the case if you are using the provided VM)
3. check that `Caffe_DIR` is set to your `caffe` `build` directory (on the VM setup this is `/home/icub/robot-code/caffe/build`)
4. check that `OpenCV_DIR` points to an `OpenCV` installation (on the VM this is `opt/ros/kinetic/share/OpenCV-3.1.0-dev`) 

#### How to create a new fine-tuning protocol

The `protocol-name` folder contains the three configuration files (`train_val.prototxt`, `solver.prototxt` and `deploy.prototxt`) that you have to fill in order to implement the desired training protocols. 
Every time that you want to implement a new training protocol, you can:

1. clone this folder into a new one and give this latter the name of the protocol you want to implement, as, e.g., shown below:
 
```sh
$ cd $LAB_DIR
$ cd assignment_dl-tuning/id_10objects_caffenet
$ cp -r protocol-name all-0
$ 
```
2. start working in the new folder by filling all points in the files where you find a `#fill#` string
3. modify the `train_and_test_net.sh` script in order to use the new folder (see next section)


#### Configure the fine-tuning script to run on your laptop:

Open the `train_and_test_net.sh` script with a text editor, e.g.:

```sh
$ cd $LAB_DIR/assignment_dl-tuning/id_10objects_caffenet
$ gedit train_and_test_net.sh
```

Check that the paths to the code and data are correct for your system. Specifically:

1. `LAB_DIR` must point to the directory of the course
2. `IMAGES_DIR` must point to the directory containing the `iCW` dataset that you downloaded. Be sure that this path ends with a `/` included, e.g. `home/icub/robot-code/datasets/iCW/`
3. the environment variable `Caffe_ROOT` is used: check that you have defined this variable in your system to point to the directory where you have cloned `caffe` (in the VM setup the variable has already been defined in `~/.bashrc-dev`)
4. at line 24 the file `bvlc_reference_caffenet.caffemodel` is used: check that after installing `caffe	` you downloaded it, as explained in the provided instructions [here](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-prepare-your-system.md#install-caffe)

Finally, this time you have to set the `PROTOCOL` variable to the name of the folder you created in previous section (around line 80 of the script). In the example above, this is:

```sh
$ PROTOCOL=all-0
```

#### Run the fine-tuning script and check results

```sh
$ cd $LAB_DIR/assignment_dl-tuning/id_10objects_caffenet
$ ./train_and_test_net.sh
```

Take a look at the logging messages and monitor the learning process: how is the loss going through iterations? Note, since we are using the `cross-entropy loss`, in the worst-case of random predictions (e.g. an output vector of size 10 and all elements set to 0.1) a reasonable value for the loss is around `-log(0.1)=2.3`.  