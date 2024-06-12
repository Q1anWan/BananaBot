%% 设置导入选项并导入数据
opts = delimitedTextImportOptions("NumVariables", 37);

% 指定范围和分隔符
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% 指定列名称和类型
opts.VariableNames = ["Timestamp", "P_LEN", "Timestamp1", "FN_L", "Timestamp2", "FN_R", "Timestamp3", "TN_L", "Timestamp4", "TN_R", "Timestamp5", "Xo0", "Timestamp6", "Xo1", "Timestamp7", "Xo2", "Timestamp8", "Xo3", "Timestamp9", "Xo4", "Timestamp10", "Xo5", "Timestamp11", "Xd2", "Timestamp12", "Xd3", "Timestamp13", "JUMP_STAGE", "Timestamp14", "P_L", "Timestamp15", "P_R", "Timestamp16", "Yawd", "Timestamp17", "Yawo", "VarName37"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% 指定变量属性
opts = setvaropts(opts, "VarName37", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "VarName37", "EmptyFieldRule", "auto");

% 导入数据
test_data = readtable("D:\WorkSpace\GraduateDesign\acc_dacc_0527_2200_yawhit.csv", opts);
time_stamp = test_data.Timestamp * 1e-6;

% 截取从 t_start 秒开始的数据
%t_start = 0;%28+6
%t_end   = 33;
t_start = 28;
t_end   = 44;

start_index = find(time_stamp >= t_start, 1);    % 找到第一个大于等于 t_start 秒的索引
end_index = find(time_stamp >= t_end, 1);

% 截取数据并从 t_start 开始调整为从 0 开始
time_s_segment = time_stamp(start_index:end_index) - t_start;

yawd_segmen = test_data.Yawd(start_index:end_index);
yawo_segmen = test_data.Yawo(start_index:end_index);

Xd2_segmen = test_data.Xd2(start_index:end_index);
Xo2_segmen = test_data.Xo2(start_index:end_index);

% 偏移 Y 数据
y_offset1 = 0;
y_offset1 = yawd_segmen(1);
y_offset2 = Xd2_segmen(1);


Y1_segment = [yawd_segmen-y_offset1,yawo_segmen-y_offset1]; % 每一列是一个 Y 数据集
Y2_segment = [Xd2_segmen-y_offset2,Xo2_segmen-y_offset2]; % 每一列是一个 Y 数据集


figure;

subplot(2,1,1);
plot(time_s_segment, Y1_segment(:, 1), '--'); % 使用实线绘制第一个数据集
hold on;
plot(time_s_segment, Y1_segment(:, 2), '-' ,Color='blue'); 
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$\psi$ [rad]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:1:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'期望航向角','观测航向角'},'Location','best');


subplot(2,1,2);
plot(time_s_segment, Y2_segment(:, 1), '--',Color='magenta'); % 使用实线绘制第一个数据集
plot(time_s_segment, Y2_segment(:, 2), '-' ,Color='red'); 
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$x$ [m]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:1:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'观测位移'});

%% 清除临时变量
clear opts