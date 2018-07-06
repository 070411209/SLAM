clc
clearvars
close all;

% SVM
load fisheriris
% 载入matlab自带的数据[有关数据的信息可以自己到UCI查找,这是UCI的经典数据之一]
% 其中meas是150*4的矩阵代表着有150个样本每个样本有4个属性描述,species代表着这150个样本的分类.
data = [meas(:,1), meas(:,2)];
% 在这里只取meas的第一列和第二列,即只选取前两个属性.
groups = ismember(species,'setosa');
% 由于species分类中是有三个分类:setosa,versicolor,virginica,
% 为了使问题简单,我们将其变为二分类问题:Setosa and non-Setosa.
[train, test] = crossvalind('holdOut',groups);
cp = classperf(groups);
% 随机选择训练集合测试集[有关crossvalind的使用请自己help一下]
% 其中cp作用是后来用来评价分类器的.
% Train support vector machine classifier
svmStruct = svmtrain(data(train,:),groups(train),'showplot',true);

% 使用svmtrain进行训练,得到训练后的结构svmStruct,在预测时使用.
% Classify using support vector machine (SVM)
classes = svmclassify(svmStruct,data(test,:),'showplot',true);

