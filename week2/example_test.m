 % Robotics: Estimation and Learning 
% WEEK 2
% 
% This script is to help run your algorithm and visualize the result from it.

%% Load data
clear all;
%close all;

load training5.mat
% This will load three variables: ball, rgb, t

% [1] ball is a 2-by-K array of ball position readings. 
%     e.g. ball(1,k) is the x position at time index k, and ball(2,k) is
%     the y position at time index k
% [2] rgb is a cell array of images containing the image of the scene at
%     the time of image recording. Only used to help you visualize the
%     scene; not used in the function
% [3] t is K-by-1 array containing time in second. (K=3701)
%     You may not use time info for implementation.

%% Plot the path of the ball ( ball의 실제 위치와 움직인 경로를 파란색 o으로 나타낸다.)
figure(1);
clf;
plot(ball(1, :), ball(2, :), 'bo-');        % ball의 위치를 나타낸다. (파란원)
hold on;
% End at red
plot(ball(1, end), ball(2, end), 's', ...   % ball의 마지막 지점을 나타낸다. (빨간 네모)
    'MarkerSize', 10, 'MarkerEdgeColor', [.5 0 0], 'MarkerFaceColor', 'r');
% start at green
plot(ball(1, 1), ball(2, 1), 's', ...       % ball의 시작점을 나타낸다. (초록 네모)
    'MarkerSize', 10, 'MarkerEdgeColor', [0 .5 0], 'MarkerFaceColor', 'g');
hold off;
axis equal;
title('Ball Position tracks');
xlabel('X (meters)');
ylabel('Y (meters)');

%% Run algorithm
% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% This overlays the predicted positions against the observed positions
state = [0, 0, 0, 0];           % state vector = [position_x, position_y, velocity_x, velocity_y]
last_t = -1;                    % 마지막 시간? 잘모르겠음
N = numel(t);                   % 관측한 시간의 횟수? 잘 모르겠음
myPredictions = zeros(2, N);    % 내가 예측한 ball의 위치를 initialization한 것.
param = {};                     % 빈 행렬? 벡터?를 만든다.
for i=1:N
    [ predictx, predicty, state, param ] = kalmanFilter( t(i), ball(1,i), ball(2,i), state, param, last_t);
    % 시간, ball의 x위치, ball의 y위치 등을 kalmanFilter함수에 입력하면 predictx predicty state param이 output된다.
    if numel(state)~=4  % state의 성분 갯수가 4개가 아니라면,
        error('Your state should be four dimensions.');
    end
    last_t = t(i);
    myPredictions(1, i) = predictx;
    myPredictions(2, i) = predicty;
end
clear px py;

%% Overlay the predictions 내가 예측한 ball의 위치를 검정색 십자가로 나타낸다.
figure(1);
hold on;
plot(myPredictions(1, :), myPredictions(2, :), 'k+-');
hold off;

%% Show the error 에러를 찾는다.
nSkip = 10;
myError = myPredictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);     % x방향, y방향의 오차를 계산한다.
                                                                    % 내가 예측한 값 - 실제값
myError_dist = sqrt(myError(1,:).^2 + myError(2,:).^2);             % 거리에 대한 오차를 계산한다. 
                                                                    % sqrt(x방향거리^2 + y방향거리^2)
myError_mean = mean(myError_dist);                                  % 오차의 평균을 계산한다.

figure(2);
clf;
plot(myError_dist);
title('Prediction Error Over Time');
xlabel('Frame');
xlim([1, numel(myError_dist)]);
ylabel('Error (meters)');
legend(sprintf('Your Prediction: %.2f mean', myError_mean));

%% Load the solution  내가 예측한 값이 아닌 Kalman Filter를 이용한 예측값이다. (솔루션)
load solution5.mat

% Error
error = predictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);
error_dist = sqrt(error(1,:).^2 + error(2,:).^2);
error_mean = mean(error_dist);
figure(2);
hold on;
plot(error_dist);
hold off;
%title(sprintf('Kalman Prediction Error: %.2f mean', error_mean));
legend(sprintf('Your Prediction: %.2f mean', myError_mean),...
    sprintf('Kalman Prediction: %.2f mean', error_mean));

figure(1);
hold on;
plot(predictions(1, :), predictions(2, :), 'mo-');
hold off;
legend('Observed','End','Start','Your Prediction','Kalman Prediction');

% figure(20);
% clf;hold on;
% plot(ball(1, 1+nSkip:end));plot(predictions(1, 1:end-nSkip));
