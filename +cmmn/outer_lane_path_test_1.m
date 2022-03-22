function path_points = outer_lane_path_test_1

% Start on the top left with the straight, cw direction
center_left_x = 1.25+.1;
center_right_x = 3.25-.1;
center_top_y = 2.75-.1;
center_bottom_y = 1.25+.1;
radius = 1-.1;

ds_lr = center_right_x - center_left_x;
ds_tb = center_top_y - center_bottom_y;
ds_qc = radius * pi/2;

yaw = [ ...
    0; ...
    0; ...
    3*pi/2; ...
    3*pi/2; ...
    pi; ...
    pi; ...
    pi/2; ...
    pi/2; ...
];
yaw(end+1) = yaw(1);

x = [ ...
    center_left_x; ...
    center_right_x; ...
    center_right_x; ...
    center_right_x; ...
    center_right_x; ...
    center_left_x; ...
    center_left_x; ...
    center_left_x; ...
];
x(end+1) = x(1);
x = x - radius * sin(yaw);

y = [ ...
    center_top_y; ...
    center_top_y; ...
    center_top_y; ...
    center_bottom_y; ...
    center_bottom_y; ...
    center_bottom_y; ...
    center_bottom_y; ...
    center_top_y; ...
];
y(end+1) = y(1);
y = y + radius * cos(yaw);

s = [ ...
    0*ds_lr + 0*ds_qc + 0*ds_tb; ...
    1*ds_lr + 0*ds_qc + 0*ds_tb; ...
    1*ds_lr + 1*ds_qc + 0*ds_tb; ...
    1*ds_lr + 1*ds_qc + 1*ds_tb; ...
    1*ds_lr + 2*ds_qc + 1*ds_tb; ...
    2*ds_lr + 2*ds_qc + 1*ds_tb; ...
    2*ds_lr + 3*ds_qc + 1*ds_tb; ...
    2*ds_lr + 3*ds_qc + 2*ds_tb; ...
    2*ds_lr + 4*ds_qc + 2*ds_tb; ...
];

assert(numel(s) == numel(x));
assert(numel(s) == numel(y));
assert(numel(s) == numel(yaw));

path_points = struct([]);
for i = 1:numel(s)
    path_points(i).pose.x = x(i);
    path_points(i).pose.y = y(i);
    path_points(i).pose.yaw = yaw(i);
    path_points(i).s = s(i);
end

end