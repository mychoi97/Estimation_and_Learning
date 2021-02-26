% Robotics: Estimation and Learning
% WEEK 3
%
% Complete this function following the instruction.
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters

% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin;

% 4. Log-odd parameters
lo_occ = param.lo_occ;
lo_free = param.lo_free;
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,
    
    % Find grids hit by the rays (in the gird map coordinate)
    occ= (repmat(ranges(:,j),1,2).*[cos(pose(3,j)+scanAngles), -sin(pose(3,j)+scanAngles)])+repmat([pose(1,j),pose(2,j)],size(ranges,1),1);
    i_occ=ceil(myResol*occ)+repmat(myorigin,1,size(ranges,1))'; % 몇번째 인덱스인지
    pos=ceil(myResol*pose(1:2,j))+myorigin; % pos 업데이트
    g_idx=unique(i_occ,'rows');
    % example : repmat(10,3,2) = [10 10;
    %                             10 10;
    %                             10 10]
    % ceil : 올림해주는 함수
    % unique : i_occ를 인덱스로 반환해줌.
    
    % Find occupied-measurement cells and free-measurement cells 
    % 뭔가 차지되어있는 공간과 비어있는 공간을 찾는다.
    for i=1:size(g_idx,1)
        % get cells in between
        [freex, freey] = bresenham(pos(1),pos(2),g_idx(i,1),g_idx(i,2));    % bresenham : 인덱스와 인덱스를 선형으로 이어주는 함수
        % convert to 1d index
        free = sub2ind(size(myMap),freey,freex);    % sub2ind는 선형인덱스로 바꿔준다.
        occupied = sub2ind(size(myMap),g_idx(i,2),g_idx(i,1));
        
        % Update the log-odds
        myMap(free) = max(myMap(free) - lo_free,lo_min); % myMap은 map을 픽셀로 나타낸 것인데, free인 곳과 occupied된 곳을 log-odds를 이용하여 나타낸다.
        myMap(occupied) =min(myMap(occupied) + lo_occ,lo_max);
        
        % Saturate the log-odd values
        
        
        % Visualize the map as needed
        
        
    end
    
end

