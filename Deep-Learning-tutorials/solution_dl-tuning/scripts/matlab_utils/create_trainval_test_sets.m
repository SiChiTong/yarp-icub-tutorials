function create_trainval_test_sets(dset_dir, setlist, output_dir)

% create/check the output directory
check_output_dir(output_dir);

if strncmp(setlist.exp_kind, 'cat', 3)
    exp_id = false;
elseif strncmp(setlist.exp_kind, 'id', 2)
    exp_id = true;
else
    error('Unsupported exp_kind in the question config file!');
end

if isfield(setlist, 'divide_trainval_perc')
    divide_trainval_perc = setlist.divide_trainval_perc;
else
    divide_trainval_perc = false;
end

if divide_trainval_perc
    validation_perc = setlist.validation_perc;
    validation_split = setlist.validation_split; % 'step' or 'random'
end

% assign labels
fid_labels = fopen(fullfile(output_dir, 'labels.txt'), 'w');
if ~exp_id
    Y_digits = containers.Map (setlist.categories, 0:(length(setlist.categories)-1));
    for line=1:numel(setlist.categories)
        fprintf(fid_labels, '%s %d\n', setlist.categories{line}, Y_digits(setlist.categories{line}));
    end
else
    obj_names = repmat(setlist.categories, length(setlist.obj_per_cat.train),1);
    obj_names = obj_names(:);
    tmp = repmat(setlist.obj_per_cat.train', length(setlist.categories),1);
    obj_names = strcat(obj_names, tmp);
    
    Y_digits = containers.Map (obj_names, 0:(length(obj_names)-1));
    for line=1:length(obj_names)
        fprintf(fid_labels, '%s %d\n', obj_names{line}, Y_digits(obj_names{line}));
    end
end
fclose(fid_labels);


% create list of all files inside the 'iCW' dir
registry = registry_from_folder(dset_dir);
[registry_nofilename, ~, registry_ext] = cellfun(@fileparts, registry, 'UniformOutput', 0);

% select only images...
registry = registry(strcmp(registry_ext, '.jpg'));
registry_nofilename = registry_nofilename(strcmp(registry_ext, '.jpg'));


%% TRAINING set

% create desired folders...
categories = setlist.categories;
instances = setlist.obj_per_cat.train;
transformations = setlist.transf_per_obj.train;
days = setlist.day_per_transf.train;
if numel(days)==2
    days = {'day1','day2','day3','day4','day5','day6','day7','day8'};
elseif strcmp(days{1},'1')
    days = {'day1', 'day3', 'day5', 'day7'};
else
    days = {'day2', 'day4', 'day6', 'day8'};
end
cameras = {'left'};

selected_branches = allcomb(categories, instances, transformations, days, cameras);
selected_branches = [selected_branches(:,1) strcat(selected_branches(:,1), selected_branches(:,2)) selected_branches(:,3:end)];
selected_branches = fullfile(selected_branches(:,1), ...
    selected_branches(:,2), ...
    selected_branches(:,3), ...
    selected_branches(:,4), ...
    selected_branches(:,5));

% select only desired folders...
registry_train = registry(ismember(registry_nofilename, selected_branches));
registry_nofilename_train = registry_nofilename(ismember(registry_nofilename, selected_branches));

% get number of frames per folder...
[registry_dirs, ~, ic] = unique(registry_nofilename_train, 'stable'); % [C,ia,ic] = unique(A) % C = A(ia) % A = C(ic)
Ndirs = size(registry_dirs,1);
Nframes = zeros(Ndirs,1);
for ii=1:Ndirs
    Nframes(ii) = sum(ic==ii);
end
% ...and its range of indices in the Y arrays
startend = zeros(Ndirs+1,1);
startend(2:end) = cumsum(Nframes);

% eventually delete exceeding frames (randomly chosen)
if setlist.limit_frames.train
    tokeep_indices = zeros(setlist.max_frames.train, Ndirs);
    for ii=1:Ndirs
        if Nframes(ii)>=setlist.max_frames.train
            tokeep_indices(:, ii) = randperm(Nframes(ii), setlist.max_frames.train) + startend(ii);
        else
            tokeep_indices(:, ii) = 1:Nframes(ii) + startend(ii);
        end
    end
    registry_train = registry_train(tokeep_indices(:), :);
end

% eventually divide train and val
if divide_trainval_perc
    
    warning('Going to create validation set by splitting the training set...');
    
    if strcmp(validation_split, 'step')
        validation_step = 1/validation_perc;
        registry_val = registry_train(1:validation_step:end,:);
        registry_train(1:validation_step:end,:) = []; % train
    elseif strcmp(validation_split, 'random')
        fidxs = randperm(length(registry_train));
        split_idx = floor(validation_perc*length(registry_train));
        registry_val = registry_train(fidxs(1:split_idx),:);
        registry_train = registry_train(fidxs(split_idx+1:end),:);
    else
        error('Wrong validation_split in config file!');
    end
    
end

% assign Y
flist_splitted = regexp(registry_train, '/', 'split');
flist_splitted = vertcat(flist_splitted{:});
if exp_id
    Y = cell2mat(values(Y_digits, flist_splitted(:,2)));
else
    Y = cell2mat(values(Y_digits, flist_splitted(:,1)));
end

if divide_trainval_perc
    flist_splitted_val = regexp(registry_val, '/', 'split');
    flist_splitted_val = vertcat(flist_splitted_val{:});
    if exp_id
        Y_val = cell2mat(values(Y_digits, flist_splitted_val(:,2)));
    else
        Y_val = cell2mat(values(Y_digits, flist_splitted_val(:,1)));
    end
end

% write output
fid_Y = fopen(fullfile(output_dir, 'train.txt'), 'w');
if divide_trainval_perc
    fid_Y_val = fopen(fullfile(output_dir, 'val.txt'), 'w');
end

for line=1:numel(registry_train)
    fprintf(fid_Y, '%s %d\n', registry_train{line}, Y(line));
end
fclose(fid_Y);

if divide_trainval_perc
    for line=1:numel(registry_val)
        fprintf(fid_Y_val, '%s %d\n', registry_val{line}, Y_val(line));
    end
    fclose(fid_Y_val);
end

%% VALIDATION set

if ~divide_trainval_perc
    
    % create desired folders...
    categories = setlist.categories;
    instances = setlist.obj_per_cat.val;
    transformations = setlist.transf_per_obj.val;
    days = setlist.day_per_transf.val;
    if numel(days)==2
        days = {'day1','day2','day3','day4','day5','day6','day7','day8'};
    elseif strcmp(days{1},'1')
        days = {'day1', 'day3', 'day5', 'day7'};
    else
        days = {'day2', 'day4', 'day6', 'day8'};
    end
    cameras = {'left'};
    
    selected_branches = allcomb(categories, instances, transformations, days, cameras);
    selected_branches = [selected_branches(:,1) strcat(selected_branches(:,1), selected_branches(:,2)) selected_branches(:,3:end)];
    selected_branches = fullfile(selected_branches(:,1), ...
        selected_branches(:,2), ...
        selected_branches(:,3), ...
        selected_branches(:,4), ...
        selected_branches(:,5));
    
    % select only desired folders...
    registry_val = registry(ismember(registry_nofilename, selected_branches));
    registry_nofilename_val = registry_nofilename(ismember(registry_nofilename, selected_branches));
    
    % get number of frames per folder...
    [registry_dirs, ~, ic] = unique(registry_nofilename_val, 'stable'); % [C,ia,ic] = unique(A) % C = A(ia) % A = C(ic)
    Ndirs = size(registry_dirs,1);
    Nframes = zeros(Ndirs,1);
    for ii=1:Ndirs
        Nframes(ii) = sum(ic==ii);
    end
    % ...and its range of indices in the Y arrays
    startend = zeros(Ndirs+1,1);
    startend(2:end) = cumsum(Nframes);
    
    % eventually delete exceeding frames (randomly chosen)
    if setlist.limit_frames.val
        tokeep_indices = zeros(setlist.max_frames.val, Ndirs);
        for ii=1:Ndirs
            if Nframes(ii)>=setlist.max_frames.val
                tokeep_indices(:, ii) = randperm(Nframes(ii), setlist.max_frames.val) + startend(ii);
            else
                tokeep_indices(:, ii) = 1:Nframes(ii) + startend(ii);
            end
        end
        registry_val = registry_val(tokeep_indices(:), :);
    end
    
    % assign Y
    flist_splitted = regexp(registry_val, '/', 'split');
    flist_splitted = vertcat(flist_splitted{:});
    if exp_id
        Y = cell2mat(values(Y_digits, flist_splitted(:,2)));
    else
        Y = cell2mat(values(Y_digits, flist_splitted(:,1)));
    end
    
    % write output
    fid_Y = fopen(fullfile(output_dir, 'val.txt'), 'w');
    for line=1:numel(registry_val)
        fprintf(fid_Y, '%s %d\n', registry_val{line}, Y(line));
    end
    fclose(fid_Y);
    
end

%% TEST set


% create desired folders...
categories = setlist.categories;
instances = setlist.obj_per_cat.test;
transformations = setlist.transf_per_obj.test;
days = setlist.day_per_transf.test;
if numel(days)==2
    days = {'day1','day2','day3','day4','day5','day6','day7','day8'};
elseif strcmp(days{1},'1')
    days = {'day1', 'day3', 'day5', 'day7'};
else
    days = {'day2', 'day4', 'day6', 'day8'};
end
cameras = {'left'};

selected_branches = allcomb(categories, instances, transformations, days, cameras);
selected_branches = [selected_branches(:,1) strcat(selected_branches(:,1), selected_branches(:,2)) selected_branches(:,3:end)];
selected_branches = fullfile(selected_branches(:,1), ...
    selected_branches(:,2), ...
    selected_branches(:,3), ...
    selected_branches(:,4), ...
    selected_branches(:,5));

% select only desired folders...
registry_test = registry(ismember(registry_nofilename, selected_branches));
registry_nofilename_test = registry_nofilename(ismember(registry_nofilename, selected_branches));

% get number of frames per folder...
[registry_dirs, ~, ic] = unique(registry_nofilename_test, 'stable'); % [C,ia,ic] = unique(A) % C = A(ia) % A = C(ic)
Ndirs = size(registry_dirs,1);
Nframes = zeros(Ndirs,1);
for ii=1:Ndirs
    Nframes(ii) = sum(ic==ii);
end
% ...and its range of indices in the Y arrays
startend = zeros(Ndirs+1,1);
startend(2:end) = cumsum(Nframes);

% eventually delete exceeding frames
if setlist.limit_frames.test
    
    tokeep_indices = zeros(setlist.max_frames.test, Ndirs);
    
    for ii=1:Ndirs
        if Nframes(ii)>=setlist.max_frames.test
            
            if ischar(setlist.test_starting_img) && strcmp(setlist.test_starting_img, 'random')
                test_starting_img = ceil(1 + (Nframes(ii)-setlist.max_frames.test-1).*rand(1,1));
            elseif isnumeric(setlist.test_starting_img) && setlist.test_starting_img>0
                test_starting_img = min(Nframes(ii)-setlist.max_frames.test, setlist.test_starting_img);
            else
                error('Unsupported test_starting_img!')
            end
            tokeep_indices(:, ii) = ( test_starting_img:(test_starting_img+setlist.max_frames.test-1) ) + startend(ii);
        else
            tokeep_indices(:, ii) = 1:Nframes(ii) + startend(ii);
        end
    end
    
    registry_test = registry_test(tokeep_indices(:), :);
end

% assign Y
flist_splitted = regexp(registry_test, '/', 'split');
flist_splitted = vertcat(flist_splitted{:});
if exp_id
    Y = cell2mat(values(Y_digits, flist_splitted(:,2)));
else
    Y = cell2mat(values(Y_digits, flist_splitted(:,1)));
end

% write output
fid_Y = fopen(fullfile(output_dir, 'test.txt'), 'w');
for line=1:numel(registry_test)
    fprintf(fid_Y, '%s %d\n', registry_test{line}, Y(line));
end
fclose(fid_Y);