%% stepper_vertical_rod_ring_torque.m
% 水平轴步进电机 + 轴承支撑 + 竖直连杆 + 顶部铁丝圆环
% 刚体动力学扭矩仿真
%
% 角度定义：
%   beta = 0 deg   ：连杆竖直向上（照片中的位置）
%   beta > 0 deg   ：连杆向一个方向偏转
%   beta = 90 deg  ：连杆水平
%
% 重要说明：
% 1. 本模型计算重力、转动惯量、轴承摩擦引起的电机轴扭矩。
% 2. 铁丝圆环按"刚性均匀细圆环"处理，不包含柔性振动和共振。
% 3. 照片中的轴承只承担径向载荷/弯矩，不会消除绕水平轴的重力扭矩。
% 4. 请把下面"用户参数"改成实测值，尤其是顶部夹具质量。
%
% MATLAB R2016b及以上可直接运行。

clear;
clc;
close all;

%% ===================== 1. 用户参数 =====================

% ---------- 几何参数 ----------
Lrod = 0.150;              % 电机轴中心到铁丝圆环连接点的杆长，m
Dring = 0.250;             % 铁丝圆环直径，m

% true：铁丝圆环在圆周最低点与连杆连接
% 此时圆环中心距离电机轴 = Lrod + Dring/2
% false：直接在下面的 dRingCenter 中填写圆环中心到电机轴的距离
ringAttachedAtBottom = true;
dRingCenter = 0.350;       % 仅在 ringAttachedAtBottom=false 时使用，m

% 圆环平面姿态：
% 0 deg  = 电机轴垂直于圆环平面（照片中通常接近这种情况），I_center=mR^2
% 90 deg = 电机轴位于圆环平面内，I_center=0.5*mR^2
ringNormalAngleDeg = 0;

% ---------- 质量参数 ----------
mRod = 0.200;              % 连杆质量，kg
mRing = 0.080;             % 铁丝圆环质量，kg

% 顶部圆柱夹具/连接件质量（照片中连杆顶部的金属圆柱）
% 请称重后填写；若它已包含在 mRod 或 mRing 中，则保持为0
mTopClamp = 0.000;         % kg
dTopClamp = Lrod;          % 夹具质心到电机轴距离，m
JTopClampLocal = 0;        % 夹具绕自身质心且平行电机轴的惯量，kg*m^2
                            % 不知道时可先设0，点质量项 m*d^2 通常占主要部分

% ---------- 电机/连接件惯量 ----------
% 17HM5417转子惯量：68 g*cm^2 = 6.8e-6 kg*m^2
JmotorRotor = 68e-7;       % kg*m^2
JshaftCoupling = 0;        % 联轴器、短轴等绕电机轴的附加惯量，kg*m^2

% ---------- 摩擦参数 ----------
% 不知道时先设为0；可通过实验逐步识别
bViscous = 0.000;          % 黏性摩擦系数，N*m*s/rad
Tcoulomb = 0.000;          % 库仑摩擦扭矩，N*m
omegaSmooth = 1e-3;        % 平滑sign函数的速度尺度，rad/s

% ---------- 运动轨迹 ----------
% beta=0表示竖直向上
betaStartDeg = 0;          % 起始角度，deg
betaEndDeg = 60;           % 终止角度，deg

holdBefore = 0.50;         % 运动前保持时间，s
moveTime = 2.00;           % 运动时间，s
holdAfter = 0.50;          % 运动后保持时间，s
sampleTime = 0.001;        % 仿真采样时间，s

% ---------- 设计安全系数 ----------
safetyFactor = 1.8;

% ---------- 电机数据 ----------
motorHoldingTorque = 0.40; % 17HM5417保持扭矩，N*m

% 下面是依据数据表曲线读取的近似动态扭矩点。
% 测试条件约为：DM420、24V、1.51A/相、3200 pulse/rev。
% 实际驱动电压、电流和加速度不同，曲线会变化。
motorCurveRPM = [0,   100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
motorCurveNm  = [0.40,0.23,0.225,0.215,0.175,0.130,0.095,0.072,0.062,0.063,0.068];

% 是否播放简化动画
enableAnimation = false;
animationStep = 30;


%% ===================== 2. 几何与惯量 =====================

g = 9.81;
Rring = Dring/2;

if ringAttachedAtBottom
    dRingCenter = Lrod + Rring;
end

% 连杆质心距离
dRodCOM = Lrod/2;

% 铁丝圆环绕自身中心、沿电机轴方向的惯量
% gamma为电机轴与圆环法线之间夹角
gamma = ringNormalAngleDeg*pi/180;
JringCenter = mRing*Rring^2*cos(gamma)^2 ...
            + 0.5*mRing*Rring^2*sin(gamma)^2;

% 各部件绕电机轴的惯量
Jrod = (1/3)*mRod*Lrod^2;
Jring = JringCenter + mRing*dRingCenter^2;
JtopClamp = JTopClampLocal + mTopClamp*dTopClamp^2;

Jload = Jrod + Jring + JtopClamp + JshaftCoupling;
Jtotal = Jload + JmotorRotor;

% 重力矩系数 Kgrav，使得重力广义力矩为 Kgrav*sin(beta)
% beta=0为竖直向上，因此重力会使系统偏离竖直向上。
Kgrav = g*(mRod*dRodCOM + mRing*dRingCenter + mTopClamp*dTopClamp);

totalMass = mRod + mRing + mTopClamp;
if totalMass > 0
    centerOfMassDistance = ...
        (mRod*dRodCOM + mRing*dRingCenter + mTopClamp*dTopClamp)/totalMass;
else
    centerOfMassDistance = 0;
end


%% ===================== 3. 五次多项式轨迹 =====================

totalTime = holdBefore + moveTime + holdAfter;
t = (0:sampleTime:totalTime).';
N = numel(t);

beta0 = betaStartDeg*pi/180;
beta1 = betaEndDeg*pi/180;
deltaBeta = beta1 - beta0;

beta = beta0*ones(N,1);
omega = zeros(N,1);
alpha = zeros(N,1);

moveMask = (t >= holdBefore) & (t <= holdBefore + moveTime);
u = (t(moveMask)-holdBefore)/moveTime;

% 五次多项式：位置、速度、加速度在两端均平滑
s = 10*u.^3 - 15*u.^4 + 6*u.^5;
dsdu = 30*u.^2 - 60*u.^3 + 30*u.^4;
d2sdu2 = 60*u - 180*u.^2 + 120*u.^3;

beta(moveMask) = beta0 + deltaBeta*s;
omega(moveMask) = deltaBeta/moveTime*dsdu;
alpha(moveMask) = deltaBeta/moveTime^2*d2sdu2;

afterMask = t > holdBefore + moveTime;
beta(afterMask) = beta1;


%% ===================== 4. 扭矩计算 =====================

% 正方向约定：
% beta正方向为电机正转方向。
% 对竖直向上的倒立摆，重力力矩会使beta继续增大：
torqueGravityOnLoad = Kgrav*sin(beta);

% 电机克服惯性所需力矩
torqueInertia = Jtotal*alpha;

% 轴承/机构摩擦所需力矩
torqueFriction = bViscous*omega ...
               + Tcoulomb*tanh(omega/omegaSmooth);

% 动力学方程：
% J*alpha = Tmotor + Tgravity - Tfriction
% 所以：
torqueMotorRequired = torqueInertia ...
                    - torqueGravityOnLoad ...
                    + torqueFriction;

% 仅看大小时，负号表示电机需要反向输出/制动
torqueRequiredAbs = abs(torqueMotorRequired);

% 转速
speedRPM = abs(omega)*60/(2*pi);

% 依据数据表近似曲线插值可用扭矩
torqueMotorAvailable = interp1( ...
    motorCurveRPM, motorCurveNm, ...
    min(speedRPM, max(motorCurveRPM)), 'linear');

% 超出数据表曲线范围时，不认为有可靠可用扭矩
outsideCurve = speedRPM > max(motorCurveRPM);
torqueMotorAvailable(outsideCurve) = 0;

% 安全系数后的需求
torqueDesignDemand = safetyFactor*torqueRequiredAbs;

% 安全系数利用率：<=1为满足
utilization = torqueDesignDemand ./ max(torqueMotorAvailable, eps);

% 实际运动路径上的最大重力扭矩
maxGravityTorqueOnPath = max(abs(torqueGravityOnLoad));

% 纯静态、任意角度下的理论最大重力扭矩（杆水平时）
maxGravityTorqueAnyAngle = Kgrav;

% 动态峰值
[peakRequiredTorque, idxPeak] = max(torqueRequiredAbs);
peakDesignTorque = safetyFactor*peakRequiredTorque;

% RMS扭矩
rmsRequiredTorque = sqrt(mean(torqueMotorRequired.^2));

% 最大速度、加速度
[maxSpeedRPM, idxMaxSpeed] = max(speedRPM);
maxAcceleration = max(abs(alpha));

% 最危险工作点
[maxUtilization, idxWorst] = max(utilization);
minimumTorqueMargin = min(torqueMotorAvailable - torqueDesignDemand);

% 保持扭矩允许的最大静态偏角
ratioNoSF = motorHoldingTorque/max(Kgrav, eps);
ratioWithSF = motorHoldingTorque/max(safetyFactor*Kgrav, eps);

betaStaticLimitNoSF = asin(min(1,ratioNoSF))*180/pi;
betaStaticLimitWithSF = asin(min(1,ratioWithSF))*180/pi;

% 最终判定
passMotorCurve = all(torqueDesignDemand <= torqueMotorAvailable + 1e-12);


%% ===================== 5. 输出结果 =====================

fprintf('\n');
fprintf('================ 水平轴-竖直杆-圆环扭矩仿真 ================\n');
fprintf('角度定义：0 deg = 连杆竖直向上；90 deg = 连杆水平\n\n');

fprintf('---------------- 几何和质量 ----------------\n');
fprintf('连杆长度：                         %.4f m\n', Lrod);
fprintf('圆环直径：                         %.4f m\n', Dring);
fprintf('圆环中心到电机轴距离：             %.4f m\n', dRingCenter);
fprintf('连杆质量：                         %.4f kg\n', mRod);
fprintf('圆环质量：                         %.4f kg\n', mRing);
fprintf('顶部夹具质量：                     %.4f kg\n', mTopClamp);
fprintf('系统总质量：                       %.4f kg\n', totalMass);
fprintf('系统质心到电机轴距离：             %.4f m\n\n', centerOfMassDistance);

fprintf('---------------- 转动惯量 ----------------\n');
fprintf('连杆转动惯量：                     %.8f kg*m^2\n', Jrod);
fprintf('圆环中心转动惯量：                 %.8f kg*m^2\n', JringCenter);
fprintf('圆环绕电机轴转动惯量：             %.8f kg*m^2\n', Jring);
fprintf('顶部夹具转动惯量：                 %.8f kg*m^2\n', JtopClamp);
fprintf('负载总转动惯量：                   %.8f kg*m^2\n', Jload);
fprintf('含电机转子的总转动惯量：           %.8f kg*m^2\n\n', Jtotal);

fprintf('---------------- 扭矩和速度 ----------------\n');
fprintf('杆水平时理论最大重力扭矩：         %.4f N*m\n', maxGravityTorqueAnyAngle);
fprintf('本次运动路径最大重力扭矩：         %.4f N*m\n', maxGravityTorqueOnPath);
fprintf('运动过程峰值电机扭矩：             %.4f N*m\n', peakRequiredTorque);
fprintf('峰值发生时间：                     %.4f s\n', t(idxPeak));
fprintf('峰值发生角度：                     %.2f deg\n', beta(idxPeak)*180/pi);
fprintf('运动过程RMS电机扭矩：              %.4f N*m\n', rmsRequiredTorque);
fprintf('运动过程最高转速：                 %.2f rpm\n', maxSpeedRPM);
fprintf('最高转速发生时间：                 %.4f s\n', t(idxMaxSpeed));
fprintf('运动过程最大角加速度：             %.4f rad/s^2\n', maxAcceleration);
fprintf('考虑%.2f倍安全系数后的峰值需求：    %.4f N*m\n\n', ...
        safetyFactor, peakDesignTorque);

fprintf('---------------- 电机校核 ----------------\n');
fprintf('电机保持扭矩：                     %.4f N*m\n', motorHoldingTorque);
fprintf('最危险点转速：                     %.2f rpm\n', speedRPM(idxWorst));
fprintf('最危险点角度：                     %.2f deg\n', beta(idxWorst)*180/pi);
fprintf('最危险点原始需求扭矩：             %.4f N*m\n', torqueRequiredAbs(idxWorst));
fprintf('最危险点安全系数后需求：           %.4f N*m\n', torqueDesignDemand(idxWorst));
fprintf('最危险点电机曲线可用扭矩：         %.4f N*m\n', torqueMotorAvailable(idxWorst));
fprintf('最大扭矩利用率：                   %.1f %%\n', 100*maxUtilization);
fprintf('最小扭矩余量：                     %.4f N*m\n', minimumTorqueMargin);
fprintf('无安全系数静态最大偏角：           %.2f deg\n', betaStaticLimitNoSF);
fprintf('按%.2f倍安全系数静态建议最大偏角：  %.2f deg\n', ...
        safetyFactor, betaStaticLimitWithSF);

if passMotorCurve
    fprintf('\n结论：按当前输入参数和近似扭矩-转速曲线，电机校核【通过】。\n');
else
    fprintf('\n结论：按当前输入参数和近似扭矩-转速曲线，电机校核【不通过】。\n');
end

if mTopClamp == 0
    fprintf(['提醒：mTopClamp当前为0。照片中的顶部金属夹具若未包含在其他质量中，' ...
             '请称重后填写，否则会低估扭矩。\n']);
end

if ringNormalAngleDeg ~= 0
    fprintf('提醒：已按圆环法线与电机轴夹角 %.2f deg 修正圆环自身惯量。\n', ...
            ringNormalAngleDeg);
end

if any(outsideCurve)
    fprintf('警告：部分转速超过数据表曲线最大转速，相关位置已判为无可靠扭矩数据。\n');
end


%% ===================== 6. 绘图 =====================

figure('Name','角度、速度和加速度','Color','w');

subplot(3,1,1);
plot(t,beta*180/pi,'LineWidth',1.5);
grid on;
xlabel('时间 / s');
ylabel('\beta / deg');
title('连杆角度（0°为竖直向上）');

subplot(3,1,2);
plot(t,omega*60/(2*pi),'LineWidth',1.5);
grid on;
xlabel('时间 / s');
ylabel('转速 / rpm');
title('电机转速');

subplot(3,1,3);
plot(t,alpha,'LineWidth',1.5);
grid on;
xlabel('时间 / s');
ylabel('角加速度 / rad/s^2');
title('角加速度');


figure('Name','扭矩随时间变化','Color','w');

plot(t,torqueInertia,'LineWidth',1.2);
hold on;
plot(t,-torqueGravityOnLoad,'LineWidth',1.2);
plot(t,torqueFriction,'LineWidth',1.2);
plot(t,torqueMotorRequired,'LineWidth',1.8);
plot(t, torqueMotorAvailable/safetyFactor, '--','LineWidth',1.5);
plot(t,-torqueMotorAvailable/safetyFactor, '--','LineWidth',1.5);
grid on;
xlabel('时间 / s');
ylabel('扭矩 / N*m');
title(sprintf('电机需求扭矩与允许范围（安全系数 %.2f）',safetyFactor));
legend('惯性项 J\alpha', ...
       '抗重力项 -T_g', ...
       '摩擦项', ...
       '电机总需求扭矩', ...
       '允许正扭矩', ...
       '允许负扭矩', ...
       'Location','best');


figure('Name','电机扭矩-转速校核','Color','w');

plot(motorCurveRPM,motorCurveNm,'o-','LineWidth',1.5);
hold on;
scatter(speedRPM,torqueDesignDemand,10,'filled');
grid on;
xlabel('电机转速 / rpm');
ylabel('扭矩 / N*m');
title('安全系数后需求工作点与电机扭矩-转速曲线');
legend('电机可用扭矩近似曲线', ...
       '安全系数后需求工作点', ...
       'Location','best');
xlim([0, max([max(motorCurveRPM), maxSpeedRPM])*1.05]);
ylim([0, max([max(motorCurveNm), max(torqueDesignDemand)])*1.15]);


figure('Name','扭矩利用率','Color','w');

plot(t,100*utilization,'LineWidth',1.5);
hold on;
yline(100,'--','100%极限','LineWidth',1.2);
grid on;
xlabel('时间 / s');
ylabel('扭矩利用率 / %');
title('安全系数后的电机扭矩利用率');


%% ===================== 7. 可选简化动画 =====================

if enableAnimation
    figure('Name','机构简化动画','Color','w');

    totalReach = max([Lrod, dRingCenter + Rring])*1.2;

    for k = 1:animationStep:N
        clf;

        b = beta(k);

        % 连杆方向：beta=0时竖直向上
        rodEndX = Lrod*sin(b);
        rodEndY = Lrod*cos(b);

        ringCenterX = dRingCenter*sin(b);
        ringCenterY = dRingCenter*cos(b);

        % 在机构转动平面内画圆环
        phi = linspace(0,2*pi,200);
        ringXlocal = Rring*cos(phi);
        ringYlocal = Rring*sin(phi);

        % 圆环随连杆整体转动
        ringX = ringCenterX + ringXlocal*cos(b) + ringYlocal*sin(b);
        ringY = ringCenterY - ringXlocal*sin(b) + ringYlocal*cos(b);

        plot([0,rodEndX],[0,rodEndY],'-','LineWidth',3);
        hold on;
        plot(ringX,ringY,'-','LineWidth',1.5);
        plot(0,0,'ko','MarkerFaceColor','k');

        axis equal;
        axis(totalReach*[-1 1 -1 1]);
        grid on;
        xlabel('水平位置 / m');
        ylabel('竖直位置 / m');
        title(sprintf('t = %.3f s,  \\beta = %.2f deg', ...
              t(k), beta(k)*180/pi));
        drawnow;
    end
end
