function registry = registry_from_folder(in_rootpath)

check_input_dir(in_rootpath);

% explore folders creating tree registry exampleCount
tree = struct('name', {}, 'subfolder', {});
[~, registry, ~,  ~] = explore_next_level_folder(in_rootpath, 0, [], '', tree);
