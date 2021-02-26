function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%   UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:  각종 변수들 정의

        dt = 1;                 % 시간 단계
        A = [1, 0, dt, 0;       % diagonal 성분에 있는 1은 이전의 상태에서의 position과 velocity를 표현하기 위함.
             0, 1, 0, dt;       % dt는 시간이 지남에 따라 위치의 변화를 나타내기 위함이다.
             0, 0, 1, 0;
             0, 0, 0, 1];
         
        H = [1, 0, 0, 0;
             0, 1, 0, 0];
         
        sig_m = 10*eye(4);
        sig_o = 0.01*eye(2);

    %% Check if the first time running this function 초기값 설정
    if previous_t < 0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);     % param.P는 그냥 P
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates 초기값 이후 계산
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
%     vx = (x - state(1)) / (t - previous_t);     % velocity =  (현재 위치 - 이전 위치)/(현재 시간 - 이전 시간)  
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;      % 현재 위치 = 이전 위치 + 속도*경과한 시간
%     predicty = y + vy * 0.330;

    P_bar = A*param.P*A' + sig_m;
    R = H*P_bar*H' + sig_o;
    K = P_bar*H'*inv(R + H*P_bar*H');
    
    temp = A*state' + K*([x; y] - H*A*state');      % state vector의 임시적인 값을 나타내는 듯? 잘 모르겠음
   
    vx = temp(3);       % x-direction velocity
    vy = temp(4);       % y-direction velocity
    predictx = temp(1) + vx*10*dt
    predicty = temp(2) + vy*10*dt
    [predictx, predicty, x, y];
    param.P = P_bar - K*H*P_bar;
    
    % State is a four dimensional element
    state = [x, y, vx, vy];
end
