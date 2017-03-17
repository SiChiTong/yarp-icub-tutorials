%% Example script to generate a recognition task

% this script uses a configuration file 
% (see e.g. ../id_10objects_ft_caffenet_icw/id_10objects.m)
% to select training, validation and test image sets
% from the iCW directory
% generating corresponding train.txt, val.txt and test.txt
% (see e.g. ../id_10objects_ft_caffenet_icw/images_lists)

% Set the root folder of the iCW dataset
IMAGES_DIR='/home/icub/robot-code/datasets/iCW';

% Set the path to current repository
TUTORIAL_DIR = '/home/icub/vvv17_deep-learning/solution_dl-tuning';

% Set the EXERCISE
EX='id_10objects_caffenet';

% Call the configuration function where the train, val, test sets are specified
% to generate a corresponding struct
% this function can be put inside the exercise folder (see the provided one):
addpath(fullfile(TUTORIAL_DIR, EX));
task_structure = id_10objects();

% Set the directory where you want to generate the 
% train.txt, val.txt, test.txt, labels.txt files
OUTPUT_DIR=fullfile(TUTORIAL_DIR, EX, 'images_lists');

%% GO!

% tell MATLAB where to find the necessary helper functions
addpath(fullfile(TUTORIAL_DIR, 'scripts'));
addpath(fullfile(TUTORIAL_DIR, 'scripts/matlab_utils'));

create_trainval_test_sets(IMAGES_DIR, task_structure, OUTPUT_DIR);
