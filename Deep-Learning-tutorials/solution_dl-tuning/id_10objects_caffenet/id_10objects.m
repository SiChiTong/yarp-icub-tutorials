function setlist = id_10objects()

%% Categories involved in the problem:
% it must be always a cell array, also with a single element
% e.g. setlist.categories = {'mug'}
% --> look in the 'iCW' folder to check the available categories
% (in this case: 'mug', 'remote', and 'sodabottle')
setlist.categories = {'mug', 'remote'};

%% Experiment kind:
% either 'cat' (obj. categorization) or 'id' (obj. identification)
setlist.exp_kind = 'id';

%% At each level of the 'iCW' hierarchy, we must define 
% one train, one validation and one test set:
% e.g. in this task we want to learn to discriminate 
% between 10 objects (5 mugs and 5 remotes)
% by training on 4 transformation sequences per object
% and testing the system on the mixed sequence 

%% Objects per category:
% if the experiment is identification, the three sets will be equal
% (like in this example)
% in categorization, instead, usually the system is trained
% on some object instances per category and tested on others
% (and usually one or two instances are kept for validation)
% --> look in the 'iCW' folder to check 
% the number of objects per category available (in this case: 10)
setlist.obj_per_cat.train= {'1', '2', '3', '4', '5'};
setlist.obj_per_cat.val = {'1', '2', '3', '4', '5'};
setlist.obj_per_cat.test = {'1', '2', '3', '4', '5'};

%% Transformations per object:
% --> look in the 'iCW' folder to check 
% the transformations per object available
setlist.transf_per_obj.train = {'ROT2D','ROT3D','SCALE','TRANSL'};
setlist.transf_per_obj.val = {'ROT2D','ROT3D','SCALE','TRANSL'};
setlist.transf_per_obj.test = {'MIX'};

%% Days per transformation:
% there are two sequences, acquired in different days, available
% you can choose to train on the first one {'1'}, or the second one {'2'}
% or both {'1','2'} (the same for testing)
% here we chose to train/validate on day 1 and test on day 2
setlist.day_per_transf.train = {'1'};
setlist.day_per_transf.val = {'1'};
setlist.day_per_transf.test = {'1'};

%% Whether to put a max limit on the #frames 
% to be sampled from each img sequence
setlist.limit_frames.train = true;
setlist.limit_frames.val = true;
setlist.limit_frames.test = true;

% Specify such limit
if setlist.limit_frames.train
    % in this example, 40 frames 
    % will be sampled RANDOMLY from each img sequence
    % to be included in the training set
    setlist.max_frames.train = 40;
end
if setlist.limit_frames.val
    % the same for validation...
    setlist.max_frames.val = 40;
end
if setlist.limit_frames.test
    setlist.max_frames.test = 200;
    % differently from training and validation,
    % the test frames are extracted as contiguous sequences
    % (like a short video on which you test the system):
    % you can start the sequence 
    % either from a random or an arbitrary frame
    % [ in the latter case, choose a number such that
    % test_starting_img + max_frames.test < 150 ]
    %setlist.test_starting_img = 'random';
    setlist.test_starting_img = 1;
    % in this case a sequence of 120 frames is extracted
    % starting from the first one
end

%% Whether to use a fraction of the training set for validation:
% this flag must be set to true in case you defined equal
% training and validation sets (like in this example,
% where we defined equal objects_per_cat, transf_per_obj and day_per_transf
% for training and validation sets)
% --> pay attention to disable the flag when it is not needed:
% (i.e., when you specify training and validation sets as different sets,
% like with different obj_per_cat, or transf_per_obj, or day_per_transf)
setlist.divide_trainval_perc = true;
if setlist.divide_trainval_perc
    
    % depending on whether we want to sample
    % validation frames at constant step or randomly
    % this field can be either 'random' or 'step'
    setlist.validation_split = 'random';
    % the validation fraction will be in (0,1)
    % NOTE: if validation_split='step' then 
    % the sampling step is computed as 1/validation_perc
    setlist.validation_perc = 0.2; 

end
