%% 设置导入选项并导入数据
opts = delimitedTextImportOptions("NumVariables", 43);

% 指定范围和分隔符
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% 指定列名称和类型
opts.VariableNames = ["Timestamp", "P_LEN", "Timestamp1", "FN_L", "Timestamp2", "FN_R", "Timestamp3", "TN_L", "Timestamp4", "TN_R", "Timestamp5", "Xo0", "Timestamp6", "Xo1", "Timestamp7", "Xo2", "Timestamp8", "Xo3", "Timestamp9", "Xo4", "Timestamp10", "Xo5", "Timestamp11", "Xd2", "Timestamp12", "Xd3", "Timestamp13", "JUMP_STAGE", "Timestamp14", "P_L", "Timestamp15", "P_R", "Timestamp16", "Yawd", "Timestamp17", "Yawo", "Timestamp18", "FN", "Timestamp19", "TN", "Timestamp20", "SKY", "VarName43"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% 指定变量属性
opts = setvaropts(opts, "VarName43", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "VarName43", "EmptyFieldRule", "auto");

% 导入数据
test_data = readtable("D:\WorkSpace\GraduateDesign\acc_dacc_0527_2400_jump.csv", opts);
time_stamp = test_data.Timestamp * 1e-6;

% 截取从 t_start 秒开始的数据
t_start = 25;
t_end   = 30;

start_index = find(time_stamp >= t_start, 1);    % 找到第一个大于等于 t_start 秒的索引
end_index = find(time_stamp >= t_end, 1);

% 截取数据并从 t_start 开始调整为从 0 开始
time_s_segment = time_stamp(start_index:end_index) - t_start;

len_segmen = test_data.P_LEN(start_index:end_index);

force_o_segmen = test_data.FN(start_index:end_index);
jump_segmen = test_data.JUMP_STAGE(start_index:end_index);


% 偏移 Y 数据
y_offset1 = 0;
y_offset2 = 0;

figure;

subplot(3,1,1);
plot(time_s_segment, len_segmen, '-' ,Color='blue'); 
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$L_p$ [m]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:0.5:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'等效摆杆长'},Location="north");


subplot(3,1,2);
plot(time_s_segment, 0.1*force_o_segmen, '-',Color='magenta'); % 使用实线绘制第一个数据集
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$F$ [Nm]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:0.5:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
legend({'沿摆推力'},Location="north");

subplot(3,1,3);
stairs(time_s_segment, jump_segmen, '-',Color='red'); % 使用实线绘制第一个数据集
hold off;
% 添加图例，并使用 LaTeX 格式
xlabel('$t$ [s]', 'Interpreter', 'latex');
ylabel('$J$ [1]', 'Interpreter', 'latex');
% 设置图形属性（可选）
grid on;                  % 添加网格
xlim([0, max(time_s_segment)]); % 设置 X 轴范围
% 设置 x 轴刻度
xticks(0:0.5:max(time_s_segment)); % 设置 x 轴间隔为 0.5 秒
%yticks(0:1:1);
legend({'跳跃标志位'},Location="north");

%% 清除临时变量
clear opts