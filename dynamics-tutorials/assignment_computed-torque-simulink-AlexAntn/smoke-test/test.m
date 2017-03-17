%Get some info
disp('Current working directory');
pwd

disp('Matlab path');
path
model = 'computed_torque';

fprintf('Loading model %s\n', model);
fprintf('In %s\n', which(model));

open_system(model,'loadonly');

fprintf('Starting model %s\n', model);
% reference = zeros(25, 1);
% reference(1) = 40;
% 
% set_param([model,'/RefDelta'], 'Value', mat2str(reference));

% This runs the model
simout = sim(model, ...
'StopTime', '10', ...
'OutputSaveName','robotState');

fprintf('Simulation finished. Quitting Matlab\n');
% %retrieving output
% robotState = simout.get('robotState');
% position = robotState.get(1);
% 
% initialConfiguration = position.Values.Data(1,:)';
% initialConfiguration = initialConfiguration + (reference * pi / 180);
% finalConfiguration = position.Values.Data(end,:)';
% 
% errorVec = abs(initialConfiguration - finalConfiguration);
% 
% maxError = 5 * pi / 180;
% 
% errors = [];
% 
% for i = 1 : length(errorVec)
%     if errorVec(i) > maxError
%         fprintf('Joint[%d]. Expected %f - Actual %f\n', i, initialConfiguration(i), finalConfiguration(i));
%         errors = [errors; i];  %#ok<AGROW>
%     end
% end
% 
% if ~isempty(errors)
%     error('Control failed to track specified reference');
% else
%     exit(0);
% end
% 
% 



quit
