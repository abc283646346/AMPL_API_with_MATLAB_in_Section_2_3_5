close all;
clc;
% 添加AMPL API路径
addpath(genpath('amplapi'));
setupOnce;
% 轨迹规划任务相关参数设置
param.K = 3;
param.Nfe = 10;
param.L_wheelbase = 2.8;
param.a_max = 0.3;
param.v_max = 2.0;
param.phy_max = 0.72;
param.w_max = 0.54;
% 声明一个ampl API对象
ampl = AMPL;
% 设置NLP问题中的参数
ampl.eval(['param Nfe := ',num2str(param.Nfe),';']);
ampl.eval(['param L_wheelbase := ',num2str(param.L_wheelbase),';']);
ampl.eval(['param a_max := ',num2str(param.a_max),';']);
ampl.eval(['param v_max := ',num2str(param.v_max),';']);
ampl.eval(['param phy_max := ',num2str(param.phy_max),';']);
ampl.eval(['param w_max := ',num2str(param.w_max),';']);
% 从文件中添加NLP模型非可变部分
ampl.read('planning.mod');
% 初始化(注：此处只是介绍初始化赋值语句写法，实际初始化方法将在2.4节介绍)
% 以x变量为例，将其中元素设置为0.01*(1+行数+列数)
x = ampl.getVariable('x');
for ii = 1 : param.Nfe
    for jj = 0 : param.K
        instance = x.get(ii,jj);
        instance.setValue(0.01 * (1 + ii + jj));
    end
end
% 指定数学规划问题求解器为IPOPT
ampl.setOption('solver','ipopt');
% 开始求解NLP问题
ampl.solve;
% 在完成求解后导出优化得到的决策变量x、y等
x = ampl.getVariable('x');
y = ampl.getVariable('y');
tf = ampl.getVariable('tf');
optimized_x = [];
optimized_y = [];
optimized_tf = tf.get().value;
for ii = 1 : param.Nfe
    for jj = 0 : param.K
        optimized_x = [optimized_x, x.get(ii,jj).value];
        optimized_y = [optimized_y, y.get(ii,jj).value];
    end
end
% 绘制优化后的轨迹
precision_level = 0.01;
[~, x] = GeneratePreciseProfileFromCollocationGrids(optimized_x, param.Nfe, optimized_tf, precision_level, 0);
[~, y] = GeneratePreciseProfileFromCollocationGrids(optimized_y, param.Nfe, optimized_tf, precision_level, 0);
figure(1)
set(0,'DefaultLineLineWidth',3);
plot(x,y,'k');
axis equal; box on; grid on; axis tight;
xlabel('x axis / m','Fontsize',16,'FontWeight','bold');
ylabel('y axis / m','Fontsize',16,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');
% 导出代价函数的优化值，并打印在屏幕上
cost_val = ampl.getObjective('cost_function'); % 需要与mod中句柄名称一致
fprintf('Cost function value = %f\n', cost_val.value);
% 切记最后要关闭AMPL对象，否则会持续占据内存资源
ampl.close;