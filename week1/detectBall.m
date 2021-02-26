% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction.

function [segI, loc] = detectBall(I)    % detectBall이라는 함수를 생성한다.

% INPUT
% I       120x160x3 numerial array  (높이(행) 120, 너비(열) 160, R,G,B 각각 1개씩)
%
% OUTPUT
% segI    120x160 numeric array     (segmented Image?인가 잘 모르겠음)
% loc     1x2 or 2x1 numeric array  (위치, 즉 좌표)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%

mu =[149.2171, 143.9402, 61.2023];                % 평균 (mean)
sigma = [13.8973, 11.5445, 18.4580];              % 표준편차 (standard deviation) (분산 variance는 sig^2이다.)
thre = 5.5e-05;                                 % 아직 뭔지 모르겠음. threshold (임계값? 문턱값?) 
covariance=[193.1341, 119.5973, -201.5219;      % 공분산 (covariance) (서로 다른 두 변수 사이의 관계를 보기 위함)
            119.5973, 133.2750, -174.9704;      % 위에서 표준편차와 분산은 하나의 변수를 위한 값들이다.
            -201.5219, -174.9704, 340.6981];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model (나의 모델을 사용해서 공 색깔의 픽셀을 찾기)
 
x_u = double(I) - cat( 3, repmat( mu(1),120,160 ), repmat( mu(2),120,160 ), repmat( mu(3),120,160 ) );
% x_u는 train 폴더에 들어있는 사진을 인풋 받고, R G B의 평균값을 빼준 것이다.
% 즉, 편차 (deviation)과 같은 역할을 한다. 평균값으로부터의 차이.
% cat(dim,A1,A2,…,An)은 차원 dim을 따라 A1, A2, … , An을 결합한다.
% 즉, 120x160x3 matrix로 된 것이다.
% repmat은 예시를 통해 설명하겠다.
% ex) repmat(10, 3, 2) = [10 10;
%                         10 10;
%                         10 10]
prob = zeros(120,160); % probability를 120x160 영행렬로 우선 initial set을 한다.
for i = 1:120
    for j = 1:160
        prob(i,j) = reshape( x_u(i,j,:),1,3 )*inv( covariance )*reshape( x_u(i,j,:),3,1 );
        % 3차원 matrix를 1x3 matrix 또는 3x1 matrix로 reshape 해준다.
    end
end
prob = (1/(124.0251*sqrt(det(covariance))))*prob;    % Correction 1/(124.0251*sqrt(det(covar))))*exp(-0.5*prob)
% prob 구하는건 아직 모르겠음.
index = find(prob > thre); % find함수를 이용해서 probability가 특정 threshold(문턱값)보다 큰 인덱스를 찾아서, 
                           % 열행렬로 나타내준다.
                           
I = true(size(prob));      % prob는 120x160 matrix니까, prob의 size는 벡터 [120 160]로 표현된다.
                           % 이제 true함수를 통해 논리값 1로 이루어진 120x160 matrix를 만든다.
                           
I(index) = false;   % Correction Flase->True
                    % 위에서 논리값 1로 이루어진 120x160 matrix를 만들었는데, 
                    % prob가 thre보다 높은 index의 경우, 논리값 0으로 바꿔준다.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
bw_biggest = false(size(I)); % I는 120x160 matrix이니까, size함수를 통해 벡터 [120 160]이 만들어진다.
                             % 그리고 false를 통해 논리값 0으로 이루어진 120x160 matrix가 만들어진다.

CC = bwconncomp(I);          % bwconncomp함수는 행렬 I에서의 연결성분들을 찾는다.
                             % 여기서 연결성분이란 : 이미지사이즈, object 개수 등이 있다. 자세한 건
                             % help bwconncomp를 이용하여 찾아봐라.
                             
numPixels = cellfun(@numel, CC.PixelIdxList); % PixelIdxList (pixel index list)
% A = cellfun(func,C)는 한 번에 셀 하나씩, 셀형 배열 C의 각 셀 내용에 함수 func를 적용합니다. 
% 그런 다음 cellfun이 func 함수의 출력값을 출력 배열 A와 결합하므로, C의 i번째 요소의 경우, A(i) = func(C{i})입니다.
% n = numel(A)는 배열 A의 요소 개수 n을 반환합니다. 이는 prod(size(A))와 동일합니다.

[biggest, idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
segI = bw_biggest;
S = regionprops(CC,'Centroid');     % regionprops함수를 이용하여, CC(I의 연결성분)의 중심을 계산한다.
loc = S(idx).Centroid;              % 중심의 index를 loc으로 정의한다.
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
