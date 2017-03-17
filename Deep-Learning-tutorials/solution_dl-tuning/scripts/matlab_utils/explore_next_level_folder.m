function [exampleCount, registry, current_path,  current_level] = explore_next_level_folder(in_rootpath, exampleCount, registry, current_path, current_level)

% get the listing of files at the current level
files = dir(fullfile(in_rootpath, current_path));

flag = 0;

for idx_file = 1:size(files)
    
    % for each folder, create its duplicate in the hierarchy
    % then get inside it and repeat recursively
    if (files(idx_file).name(1)~='.')
        
        if (files(idx_file).isdir)
            
            tmp_path = current_path;
            current_path = fullfile(current_path, files(idx_file).name);
            
            current_level(length(current_level)+1).name = files(idx_file).name;
            current_level(length(current_level)).subfolder = struct('name', {}, 'subfolder', {});
            [exampleCount, registry, ~, current_level(length(current_level)).subfolder] = explore_next_level_folder(in_rootpath, exampleCount, registry, current_path, current_level(length(current_level)).subfolder);
            
            % fall back to the previous level
            current_path = tmp_path;
            
        else
            
            if flag==0 
                flag = 1;
                tobeadded = 1;
            end
            
            if tobeadded
                file_src = fullfile(current_path, files(idx_file).name);
                exampleCount = exampleCount + 1;
                registry{exampleCount,1} = file_src; 
            end
            
        end
    end
end