function plot_log(working_dir)

% These snippets serve as basic examples.
% You can copy, paste, edit them in whatever way you want.
% Check the data files before designing your plots!

% Generate the necessary data files with 
% parse_caffe_log.sh before plotting:
% (see train_and_test_net.sh as example usage)
% Once you have generated caffeINFOtrain.txt and caffeINFOval.txt
% inside your working_dir, you can call: 
% plot_log(working_dir)

% The fields present in the data files that are usually proper to plot along
% the y axis are test accuracy, test loss, training loss, and learning rate.
% Those should plot along the x axis are training iterations and/or seconds.
% Possible combinations implemented below:
% 1. Test accuracy vs. training iterations;
% 2. Test loss vs. training iterations;
% 3. Training accuracy vs. training iterations;
% 4. Training loss vs. training iterations;
% 5. Learning rate vs. training iterations;

fid = fopen(fullfile(working_dir, 'caffeINFOtrain.txt'));
train_data = textscan(fid, '%d %f %f', 'Delimiter', '\t', 'CommentStyle','#');
fclose(fid);

fid = fopen(fullfile(working_dir, 'caffeINFOval.txt'));
val_data = textscan(fid, '%d %f %f %f', 'Delimiter', '\t', 'CommentStyle','#');
fclose(fid);

%%%%%% fields in the data file caffeINFOtrain.txt are
%%%%%% iter acc loss
train_acc = train_data{2};
train_loss = train_data{3};

%%%%%% fields in the data file caffeINFOval.txt are
%%%%%% iter acc loss lr
iter = val_data{1};
val_acc = val_data{2};
val_loss = val_data{3};
lr = val_data{4};

val_color = [0 204/255 0];
train_color = [255/255 0 0];

figure
plot(iter,val_acc, 'o', 'MarkerSize', 3, 'MarkerFace', val_color, 'MarkerEdge', val_color)
hold on 
plot(iter,train_acc, 'o', 'MarkerSize', 2, 'MarkerFace', train_color, 'MarkerEdge', train_color)
legend({'val acc','train acc'})
title('Accuracy vs. Training Iterations')
xlabel('Training Iterations')
ylabel('Accuracy')
grid on
box on
saveas(gcf, fullfile(working_dir,'matlab_caffeINFO_acc.png'))

figure
plot(iter,val_loss, 'o', 'MarkerSize', 3, 'MarkerFace', val_color, 'MarkerEdge', val_color)
hold on 
plot(iter,train_loss, 'o', 'MarkerSize', 2, 'MarkerFace', train_color, 'MarkerEdge', train_color)
legend({'val loss','train loss'})
title('Loss vs. Training Iterations')
xlabel('Training Iterations')
ylabel('Loss')
grid on
box on
saveas(gcf, fullfile(working_dir,'matlab_caffeINFO_loss.png'))

figure
plot(iter,lr, 'o', 'MarkerSize', 2, 'MarkerFace', train_color, 'MarkerEdge', train_color)
title('Learning Rate vs. Training Iterations')
xlabel('Training Iterations')
ylabel('Learning Rate')
grid on
box on
saveas(gcf, fullfile(working_dir,'matlab_caffeINFO_lr.png'))



