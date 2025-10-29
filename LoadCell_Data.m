% Before Using: 
% Ensure the data is loaded correctly and check for any missing values
% i.e., save csv file from STM32 as 'load_cell_array.csv' and save in
% this workspace :)

data = readmatrix('load_cell_array.csv');
plot(data(:,1)/1000, data(:,2)); % time (s) vs weight (g)
xlabel("Time (s)");
ylabel("Weight (g)");