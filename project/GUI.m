clc
clear variables

% connect to arduino
delete(instrfind({'Port'},{'COM6'}));
s = serial('COM6');
fopen(s);


for(i = 1:100)
    fscanf(s)
end
