%% 设置导入选项并导入数据
opts = delimitedTextImportOptions("NumVariables", 33);

% 指定范围和分隔符
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% 指定列名称和类型
opts.VariableNames = ["Timestamp", "P_LEN", "Timestamp1", "FN_L", "Timestamp2", "FN_R", "Timestamp3", "TN_L", "Timestamp4", "TN_R", "Timestamp5", "Xo0", "Timestamp6", "Xo1", "Timestamp7", "Xo2", "Timestamp8", "Xo3", "Timestamp9", "Xo4", "Timestamp10", "Xo5", "Timestamp11", "Xd2", "Timestamp12", "Xd3", "Timestamp13", "JUMP_STAGE", "Timestamp14", "P_L", "Timestamp15", "P_R", "VarName33"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% 指定变量属性
opts = setvaropts(opts, "VarName33", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "VarName33", "EmptyFieldRule", "auto");

% 导入数据
test_data = readtable("D:\WorkSpace\GraduateDesign\acc_dacc_0527_2100_single.csv", opts);
time_stamp = test_data.Timestamp * 1e-6;

% 截取从 t_start 秒开始的数据
t_start = 15;
t_end   = t_start + 46.2-15;
%t_start = 0;
%t_end   = time_stamp(end);
start_index = find(time_stamp >= t_start, 1);    % 找到第一个大于等于 t_start 秒的索引
end_index = find(time_stamp >= t_end, 1);

% 截取数据并从 t_start 开始调整为从 0 开始
time_s_segment = time_stamp(start_index:end_index) - t_start;
Xo2_segment = test_data.Xo2(start_index:end_index);
Xd2_segment = test_data.Xd2(start_index:end_index);

Xo3_segment = test_data.Xo3(start_index:end_index);
Xd3_segment = test_data.Xd3(start_index:end_index);

% 偏移 Y 数据
y_offset = Xd2_segment(1);

% 将多个 Y 数据集组合成一个矩阵
Y1_segment = [Xd2_segment-y_offset,Xo2_segment-y_offset]; % 每一列是一个 Y 数据集
Y2_segment = [Xd3_segment,Xo3_segment]; % 每一列是一个 Y 数据集

figure;

subplot(2,1,1);
plot(time_s_segment, Y1_segment(:, 1), '--'); % 使用实线绘制第一个数据集
hold on;
plot(time_s_segment, Y1_segment(:, 2), '-' ,Color='blue'); % 使用虚线绘制第二个数据集
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$x$ [m]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:2:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'$x_d$','$\hat{x}$'}, 'Interpreter', 'latex');


subplot(2,1,2);
plot(time_s_segment, Y2_segment(:, 1), '--',Color='magenta'); % 使用实线绘制第一个数据集
hold on;
plot(time_s_segment, Y2_segment(:, 2), '-',Color='red'); % 使用虚线绘制第二个数据集
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$\dot{x}$ [m/s]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:2:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'$\dot{x}_d$','$\hat{\dot{x}}$'}, 'Interpreter', 'latex');

%% 清除临时变量
clear opts