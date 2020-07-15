

clc; clear; close all;

%% add path of DeepLearnToolbox
addpath(genpath('ImageProcessing\DeepLearnToolbox_trimmed'));
%% load MNIST dataset
load mnist_uint8;
%% 格式化数据集
train_x = double(reshape(train_x',28,28,[]))/255;
train_x = permute(train_x, [2 1 3]);
train_y = double(train_y');

test_x = double(reshape(test_x',28,28,[]))/255;
test_x = permute(test_x, [2 1 3]);
test_y = double(test_y');

%% 建立卷积神经网络
cnn.layers = {
    %输入层
    struct('type', 'i')    
    %卷积层
    struct('type', 'c', 'outputmaps', 6, 'kernelsize', 5)    
    %平均池化层
    struct('type', 's', 'scale', 2)    
    %卷积层
    struct('type', 'c', 'outputmaps', 12, 'kernelsize', 5)    
    %平均池化层
    struct('type', 's', 'scale', 2) 
};
cnn = cnnsetup(cnn, train_x, train_y);

%% 训练CNN
%学习速率：更新内核的速率
learning_rate = 1; 
% 批量大小：每次更新时输入网络的图像数  
batch_size = 50; 
%阶段数：整个数据集上训练的迭代次数
%1个历元的错误率约为11%，100个历元的错误率约为1.2% 
num_epochs = 1; 

opts.alpha = learning_rate;
opts.batchsize = batch_size;
opts.numepochs = num_epochs;

%在反向传播中使用经典梯度下降
cnn = cnntrain(cnn, train_x, train_y, opts);
%% 测试 CNN
[error_rate, cnn, bad] = cnntest(cnn, test_x, test_y);
fprintf('Error rate = %.2f%%\n', error_rate*100);
%% 训练过程中逐批绘制均方误差图 
figure('numberTitle', 'off', 'name', ...
    '训练中的批处理均方差'); 
plot(cnn.rL, 'linewidth', 2);
xlabel('批次')
ylabel('均方差')
set(gca, 'fontsize', 16, ...
    'xlim', [0 length(cnn.rL)], ...
    'ylim', [0 ceil(max(cnn.rL)*10)*.1]);
grid on

%% 分类精度
[~, labels] = max(test_y);
h1 = hist(labels, size(test_y,1));
labels(bad) = [];
h2 = hist(labels, size(test_y,1));
figure('numberTitle', 'off', 'name', ...
    '分类精度'); hold on
colors = get(gca, 'colororder');
bar(0:9, h1, .85, 'facecolor', colors(1,:))
bar(0:9, h2, .80, 'facecolor', colors(2,:))
legend('测试样本的数量', ...
    '正确分类样本数量', 'location', 'southeast')
set(gca, ...
    'ygrid', 'on', ...
    'fontsize', 14, ...
    'xlim', [-.5 9.5], ...
    'xtick', 0:9, ...
    'ylim', [0 1200] ...
    )
accuracy = h2 ./ h1;
for i = 1:length(accuracy)
    str = sprintf('%.2f', accuracy(i));
    text(i-1.4, h1(i)+30, str, 'fontsize', 14);
end

%% 卷积核的可视化
for i = 1:length(cnn.layers)
    if cnn.layers{i}.type ~= 'c'
        continue
    end
    kernels = cnn.layers{i}.k;
    num_input_maps = length(kernels);
    num_output_maps = length(kernels{1});
    figure('numberTitle', 'off', 'name', ...
        sprintf('卷积层内核： %d input -> %d output', ...
        num_input_maps, num_output_maps));
    for j = 1:num_input_maps
        for k = 1:num_output_maps
            subplot(num_input_maps, num_output_maps, ...
                (j-1)*num_output_maps+k)
            imagesc(kernels{j}{k})
            colormap gray
            axis image
            set(gca, ...
                'xticklabel', [], ...
                'yticklabel', [], ...
                'ticklength', [0 0])
        end
    end
end

%% 用训练网络进行前馈的一个例子
figure('numberTitle', 'off', 'name', ...
    '用训练网络进行前馈的一个例子');
num_layers = length(cnn.layers);
n_rows = length(cnn.layers{end}.b);
n_cols = num_layers + 2;
sample_ind = 1; % 1~batch_size
value_range = [0 1];
for i = 1:num_layers
    switch cnn.layers{i}.type
        % 显示输入图像
        case 'i'
            subplot(n_rows, n_cols, 1:n_cols:(n_rows*n_cols))
            img = cnn.layers{i}.a{1}(:,:,sample_ind);
            title_str = '输入';
            imagesc(img, value_range);
            colormap gray
            axis image
            set(gca, ...
                'xticklabel', [], ...
                'yticklabel', [], ...
                'ticklength', [0 0])
            title(title_str, 'fontsize', 12)
            
        % 卷积后的图像
        case 'c'
            img_size = size(cnn.layers{i}.a{1});
            title_str = '卷积层';
            span = n_rows / length(cnn.layers{i}.a);
            for j = 1:length(cnn.layers{i}.a)
                locs = [];
                for k = 1:span 
                    locs = [locs (j-1)*span*n_cols+(k-1)*n_cols+i];
                end
                subplot(n_rows, n_cols, locs)
                img = cnn.layers{i}.a{j}(:,:,sample_ind);
                imagesc(img, value_range);
                colormap gray
                axis image
                set(gca, ...
                    'xticklabel', [], ...
                    'yticklabel', [], ...
                    'ticklength', [0 0])
                if j == 1
                    title(title_str, 'fontsize', 12)
                end
            end
                
        % 池化后的图像       
        case 's'
            img_size = size(cnn.layers{i}.a{1});
            title_str = '池化层';
            span = n_rows / length(cnn.layers{i}.a);
            for j = 1:length(cnn.layers{i}.a)
                locs = [];
                for k = 1:span 
                    locs = [locs (j-1)*span*n_cols+(k-1)*n_cols+i];
                end
                subplot(n_rows, n_cols, locs)
                img = cnn.layers{i}.a{j}(:,:,sample_ind);
                imagesc(img, value_range);
                colormap gray
                axis image
                set(gca, ...
                    'xticklabel', [], ...
                    'yticklabel', [], ...
                    'ticklength', [0 0])
                if j == 1
                    title(title_str, 'fontsize', 12)
                end
            end
    end
end

% 完全连接层后的图像
subplot(n_rows, n_cols, (n_cols-1):n_cols:(n_rows*n_cols))        
img = cnn.fv(:,sample_ind);
imagesc(img, value_range);
colormap gray
axis image
set(gca, ...
    'xticklabel', [], ...
    'yticklabel', [], ...
    'ticklength', [0 0])
title('完全连接层', 'fontsize', 12)

% 输出图像
subplot(n_rows, n_cols, n_cols:n_cols:(n_rows*n_cols))        
img = cnn.o(:,sample_ind);
imagesc(img, value_range);
colormap gray
axis image
set(gca, ...
    'xticklabel', [], ...
    'yticklabel', 0:9, ...
    'ticklength', [0 0], ...
    'fontsize', 12, ...
    'ytick', 1:10 ...
    )
title('输出', 'fontsize', 12)
