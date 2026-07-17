%% stepper_vertical_rod_ring_torque_revised.m
% 水平轴步进电机 + 轴承支撑 + 竖直连杆 + 顶部铁丝圆环
% 刚体动力学扭矩仿真（修订版）
%
% 角度定义：
%   beta = 0 deg   ：连杆竖直向上
%   beta > 0 deg   ：连杆向设定正方向偏转
%   beta = 90 deg  ：连杆水平
%
% 本修订版主要改动：
% 1. 圆环质心距离默认采用实测值，不再被 ringAttachedAtBottom 静默覆盖。
% 2. 分开输出"静态重力利用率"和"动态总利用率"。
% 3. 输出最危险点的重力、惯性、摩擦扭矩分解。
% 4. 明确说明利用率是"理论需求/录入的电机曲线"，不是电流百分比。
% 5. 增加参数检查、扭矩标定系数和详细告警。
% 6. 绘图不使用 yline，兼容 MATLAB R2016b。
%
% 重要说明：
% 1. 本模型计算重力、转动惯量和机构摩擦引起的电机轴扭矩。
% 2. 铁丝圆环按刚性均匀细圆环处理，不包含柔性振动、共振和连接间隙。
% 3. 轴承可承担径向载荷和弯矩，但不会自动消除绕水平轴的重力扭矩。
% 4. 质量和质心距离必须尽量使用实测值。
% 5. 步进电机扭矩曲线会随驱动器、电压、电流、细分和温升变化。
%
% MATLAB R2016b及以上可直接运行。

clear;
clc;
close all;

%% ===================== 1. 用户参数 =====================

% ---------- 几何参数 ----------
Lrod = 0.220;              % 电机轴中心到圆环连接位置的杆长，m
Dring = 0.400;             % 铁丝圆环直径，m

% 圆环中心距离的确定方式：
%   'measured'：直接使用实测的 dRingCenterMeasured（推荐）
%   'geometry'：按 dRingCenter = Lrod + Dring/2 自动计算
ringCenterDistanceMode = 'measured';
dRingCenterMeasured = 0.350;   % 电机轴中心到整个圆环质心的实测距离，m

% 圆环平面姿态：
% 0 deg  = 电机轴垂直于圆环平面，I_center = mR^2
% 90 deg = 电机轴位于圆环平面内，I_center = 0.5mR^2
ringNormalAngleDeg = 0;

% ---------- 质量参数 ----------
mRod = 0.200;              % 连杆质量，kg（0.200 kg = 200 g）
mRing = 0.080;             % 铁丝圆环质量，kg（0.080 kg = 80 g）

% 顶部圆柱夹具/连接件
% 若夹具质量已包含在 mRod 或 mRing 中，则保持为0，避免重复计算。
mTopClamp = 0.000;         % kg
dTopClamp = Lrod;          % 夹具质心到电机轴距离，m
JTopClampLocal = 0;        % 夹具绕自身质心且平行于电机轴的惯量，kg*m^2

% ---------- 电机、联轴器和转轴惯量 ----------
% 17HM5417转子惯量：68 g*cm^2 = 6.8e-6 kg*m^2
JmotorRotor = 68e-7;       % kg*m^2
JshaftCoupling = 0;        % 联轴器、短轴等绕电机轴的附加惯量，kg*m^2

% ---------- 摩擦参数 ----------
% 不知道时可先设为0，再通过实验辨识。
bViscous = 0.000;          % 黏性摩擦系数，N*m*s/rad
Tcoulomb = 0.000;          % 运动时库仑摩擦扭矩，N*m
omegaSmooth = 1e-3;        % 平滑sign函数速度尺度，rad/s

% ---------- 运动轨迹 ----------
% beta=0表示竖直向上。
betaStartDeg = 0;          % 起始角度，deg
betaEndDeg = 60;           % 终止角度，deg

holdBefore = 0.50;         % 运动前保持时间，s
moveTime = 2.00;           % 运动时间，s
holdAfter = 0.50;          % 运动后保持时间，s
sampleTime = 0.001;        % 仿真采样时间，s

% ---------- 设计安全系数 ----------
safetyFactor = 1.00;

% ---------- 电机数据 ----------
motorHoldingTorque = 0.40; % 标称保持扭矩，N*m

% 电机扭矩标定系数：
% 1.00 = 完全使用下面录入的标称曲线。
% 若以后通过拉力计实测确认电机实际扭矩更大或更小，可统一修正。
% 例如实测比曲线高10%，可填1.10；没有实测依据时保持1.00。
motorTorqueScale = 1.00;

% 依据数据表读取的近似动态扭矩点。
% 原参考条件约为：DM420、24 V、1.51 A/相、3200 pulse/rev。
% 实际驱动器、电压、电流、细分、温升不同，曲线会变化。
motorCurveRPM = [0,    100,  200,  300,  400,  500,  600,  700,  800,  900, 1000];
motorCurveNm  = [0.40, 0.23, 0.225,0.215,0.175,0.130,0.095,0.072,0.062,0.063,0.068];

% ---------- 输出和动画 ----------
enableAnimation = false;
animationStep = 30;
exportCSV = false;         % true时导出全部时序结果CSV
csvFileName = 'stepper_torque_simulation_results.csv';


%% ===================== 2. 参数检查 =====================

if Lrod <= 0
    error('Lrod必须大于0。');
end
if Dring <= 0
    error('Dring必须大于0。');
end
if dRingCenterMeasured < 0
    error('dRingCenterMeasured不能小于0。');
end
if any([mRod, mRing, mTopClamp] < 0)
    error('质量参数不能为负数。');
end
if any([JmotorRotor, JshaftCoupling, JTopClampLocal] < 0)
    error('转动惯量参数不能为负数。');
end
if moveTime <= 0 || sampleTime <= 0
    error('moveTime和sampleTime必须大于0。');
end
if holdBefore < 0 || holdAfter < 0
    error('保持时间不能为负数。');
end
if safetyFactor <= 0
    error('safetyFactor必须大于0。');
end
if motorHoldingTorque <= 0 || motorTorqueScale <= 0
    error('电机保持扭矩和扭矩标定系数必须大于0。');
end
if omegaSmooth <= 0
    error('omegaSmooth必须大于0。');
end
if numel(motorCurveRPM) ~= numel(motorCurveNm) || numel(motorCurveRPM) < 2
    error('motorCurveRPM与motorCurveNm长度必须相同，且至少包含两个点。');
end
if any(diff(motorCurveRPM) <= 0)
    error('motorCurveRPM必须严格递增。');
end
if motorCurveRPM(1) ~= 0
    error('motorCurveRPM第一个点应为0 rpm。');
end
if any(motorCurveNm <= 0)
    error('motorCurveNm中的扭矩必须全部大于0。');
end
if ~strcmpi(ringCenterDistanceMode,'measured') && ...
   ~strcmpi(ringCenterDistanceMode,'geometry')
    error('ringCenterDistanceMode只能是''measured''或''geometry''。');
end


%% ===================== 3. 几何、质量与惯量 =====================

g = 9.81;
Rring = Dring/2;

if strcmpi(ringCenterDistanceMode,'measured')
    dRingCenter = dRingCenterMeasured;
    ringDistanceDescription = '采用实测值';
else
    dRingCenter = Lrod + Rring;
    ringDistanceDescription = '按 Lrod + Dring/2 自动计算';
end

% 连杆按均匀细杆处理，质心位于中点。
dRodCOM = Lrod/2;

% 铁丝圆环绕自身中心、沿电机轴方向的惯量。
% gamma为电机轴与圆环法线之间夹角。
gamma = ringNormalAngleDeg*pi/180;
JringCenter = mRing*Rring^2*cos(gamma)^2 ...
            + 0.5*mRing*Rring^2*sin(gamma)^2;

% 各部件绕电机轴的惯量。
Jrod = (1/3)*mRod*Lrod^2;
Jring = JringCenter + mRing*dRingCenter^2;
JtopClamp = JTopClampLocal + mTopClamp*dTopClamp^2;

Jload = Jrod + Jring + JtopClamp + JshaftCoupling;
Jtotal = Jload + JmotorRotor;

% 各部件的最大重力矩系数，水平时达到最大。
KgravRod = g*mRod*dRodCOM;
KgravRing = g*mRing*dRingCenter;
KgravTopClamp = g*mTopClamp*dTopClamp;
Kgrav = KgravRod + KgravRing + KgravTopClamp;

% 总质心位置。
totalMass = mRod + mRing + mTopClamp;
if totalMass > 0
    centerOfMassDistance = ...
        (mRod*dRodCOM + mRing*dRingCenter + mTopClamp*dTopClamp)/totalMass;
else
    centerOfMassDistance = 0;
end


%% ===================== 4. 五次多项式轨迹 =====================

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

% 五次多项式：位置、速度和加速度在运动两端均为平滑过渡。
s = 10*u.^3 - 15*u.^4 + 6*u.^5;
dsdu = 30*u.^2 - 60*u.^3 + 30*u.^4;
d2sdu2 = 60*u - 180*u.^2 + 120*u.^3;

beta(moveMask) = beta0 + deltaBeta*s;
omega(moveMask) = deltaBeta/moveTime*dsdu;
alpha(moveMask) = deltaBeta/moveTime^2*d2sdu2;

beforeMask = t < holdBefore;
afterMask = t > holdBefore + moveTime;
beta(beforeMask) = beta0;
beta(afterMask) = beta1;


%% ===================== 5. 扭矩计算 =====================

% 正方向约定：beta正方向为电机正转方向。
% beta=0为竖直向上的不稳定平衡点，重力会促使|beta|继续增大。
torqueGravityOnLoad = Kgrav*sin(beta);

% 电机为实现设定角加速度所需的惯性项。
torqueInertia = Jtotal*alpha;

% 机构运动摩擦项。
torqueFriction = bViscous*omega ...
               + Tcoulomb*tanh(omega/omegaSmooth);

% 动力学方程：
%   J*alpha = Tmotor + Tgravity - Tfriction
% 因此电机所需扭矩：
%   Tmotor = J*alpha - Tgravity + Tfriction
torqueMotorRequired = torqueInertia ...
                    - torqueGravityOnLoad ...
                    + torqueFriction;

torqueRequiredAbs = abs(torqueMotorRequired);

% 电机转速。
speedRPM = abs(omega)*60/(2*pi);

% 应用实测标定系数后的电机能力。
effectiveHoldingTorque = motorHoldingTorque*motorTorqueScale;
effectiveMotorCurveNm = motorCurveNm*motorTorqueScale;

% 根据录入的扭矩-转速曲线插值。
curveMaxRPM = max(motorCurveRPM);
queryRPM = min(speedRPM,curveMaxRPM);
torqueMotorAvailable = interp1( ...
    motorCurveRPM,effectiveMotorCurveNm,queryRPM,'linear');

% 超出曲线范围时没有可靠数据，按0处理。
outsideCurve = speedRPM > curveMaxRPM;
torqueMotorAvailable(outsideCurve) = 0;

% 安全系数后的设计需求。
torqueDesignDemand = safetyFactor*torqueRequiredAbs;

% 动态总利用率：理论设计需求 / 当前转速下录入的可用扭矩。
utilizationTotal = torqueDesignDemand ./ max(torqueMotorAvailable,eps);

% 静态重力利用率：只看重力，分母使用有效保持扭矩。
torqueStaticGravityDemand = safetyFactor*abs(torqueGravityOnLoad);
utilizationStaticGravity = torqueStaticGravityDemand/max(effectiveHoldingTorque,eps);

% 动态附加扭矩：总需求与单纯抗重力需求之差，仅用于帮助理解。
% 该量可能为正或负，不能单独作为电机选型依据。
torqueMotorForStaticBalance = -torqueGravityOnLoad;
torqueDynamicAndFriction = torqueMotorRequired - torqueMotorForStaticBalance;


%% ===================== 6. 统计和校核 =====================

% 重力矩。
maxGravityTorqueOnPath = max(abs(torqueGravityOnLoad));
maxGravityTorqueAnyAngle = Kgrav;

% 总需求峰值。
[peakRequiredTorque,idxPeak] = max(torqueRequiredAbs);
peakDesignTorque = safetyFactor*peakRequiredTorque;

% RMS扭矩。
rmsRequiredTorque = sqrt(mean(torqueMotorRequired.^2));

% 最大速度和加速度。
[maxSpeedRPM,idxMaxSpeed] = max(speedRPM);
maxAcceleration = max(abs(alpha));

% 全过程最危险工作点。
[maxUtilization,idxWorst] = max(utilizationTotal);
minimumTorqueMargin = min(torqueMotorAvailable - torqueDesignDemand);

% 仅运动阶段最危险点。
actualMovingMask = moveMask & ...
    ((abs(omega) > 1e-12) | (abs(alpha) > 1e-12));
if any(actualMovingMask)
    movingIndices = find(actualMovingMask);
    [maxMovingUtilization,idxLocalMovingWorst] = ...
        max(utilizationTotal(actualMovingMask));
    idxMovingWorst = movingIndices(idxLocalMovingWorst);
else
    maxMovingUtilization = 0;
    idxMovingWorst = 1;
end

% 静态重力最危险点。
[maxStaticGravityUtilization,idxWorstStatic] = ...
    max(utilizationStaticGravity);

% 起点、终点静态保持需求。
startStaticTorque = abs(Kgrav*sin(beta0));
endStaticTorque = abs(Kgrav*sin(beta1));
startStaticDesignTorque = safetyFactor*startStaticTorque;
endStaticDesignTorque = safetyFactor*endStaticTorque;
startStaticUtilization = startStaticDesignTorque/max(effectiveHoldingTorque,eps);
endStaticUtilization = endStaticDesignTorque/max(effectiveHoldingTorque,eps);

% 保持扭矩允许的最大静态偏角。
ratioNoSF = effectiveHoldingTorque/max(Kgrav,eps);
ratioWithSF = effectiveHoldingTorque/max(safetyFactor*Kgrav,eps);
betaStaticLimitNoSF = asin(min(1,ratioNoSF))*180/pi;
betaStaticLimitWithSF = asin(min(1,ratioWithSF))*180/pi;

% 最终判定。
passMotorCurve = all(torqueDesignDemand <= torqueMotorAvailable + 1e-12);
passEndStatic = endStaticDesignTorque <= effectiveHoldingTorque + 1e-12;

% 超过100%的持续时间。
overloadMask = utilizationTotal > 1;
overloadDuration = sum(overloadMask)*sampleTime;
if any(overloadMask)
    idxFirstOverload = find(overloadMask,1,'first');
    idxLastOverload = find(overloadMask,1,'last');
else
    idxFirstOverload = 1;
    idxLastOverload = 1;
end


%% ===================== 7. 输出结果 =====================

fprintf('\n');
fprintf('================ 水平轴-竖直杆-圆环扭矩仿真（修订版） ================\n');
fprintf('角度定义：0 deg = 连杆竖直向上；90 deg = 连杆水平\n');
fprintf('利用率定义：理论需求扭矩 / 录入或标定后的电机可用扭矩\n');
fprintf('注意：利用率不是电流百分比，也不是电机内部真实应力的直接测量值。\n\n');

fprintf('---------------- 几何和质量 ----------------\n');
fprintf('连杆长度：                         %.4f m\n',Lrod);
fprintf('圆环直径：                         %.4f m\n',Dring);
fprintf('圆环质心距离模式：                 %s\n',ringDistanceDescription);
fprintf('圆环中心到电机轴距离：             %.4f m\n',dRingCenter);
if strcmpi(ringCenterDistanceMode,'measured')
    fprintf('按几何推算的对比距离：             %.4f m\n',Lrod + Rring);
end
fprintf('连杆质量：                         %.4f kg  (%.1f g)\n',mRod,1000*mRod);
fprintf('圆环质量：                         %.4f kg  (%.1f g)\n',mRing,1000*mRing);
fprintf('顶部夹具质量：                     %.4f kg  (%.1f g)\n',mTopClamp,1000*mTopClamp);
fprintf('系统总质量：                       %.4f kg\n',totalMass);
fprintf('系统质心到电机轴距离：             %.4f m\n\n',centerOfMassDistance);

fprintf('---------------- 各部件最大重力矩贡献 ----------------\n');
fprintf('连杆水平时重力矩贡献：             %.4f N*m\n',KgravRod);
fprintf('圆环水平时重力矩贡献：             %.4f N*m\n',KgravRing);
fprintf('顶部夹具水平时重力矩贡献：         %.4f N*m\n',KgravTopClamp);
fprintf('全系统水平时总重力矩：             %.4f N*m\n\n',Kgrav);

fprintf('---------------- 转动惯量 ----------------\n');
fprintf('连杆转动惯量：                     %.8f kg*m^2\n',Jrod);
fprintf('圆环中心转动惯量：                 %.8f kg*m^2\n',JringCenter);
fprintf('圆环绕电机轴转动惯量：             %.8f kg*m^2\n',Jring);
fprintf('顶部夹具转动惯量：                 %.8f kg*m^2\n',JtopClamp);
fprintf('负载总转动惯量：                   %.8f kg*m^2\n',Jload);
fprintf('含电机转子的总转动惯量：           %.8f kg*m^2\n\n',Jtotal);

fprintf('---------------- 电机能力设定 ----------------\n');
fprintf('标称保持扭矩：                     %.4f N*m\n',motorHoldingTorque);
fprintf('扭矩标定系数：                     %.4f\n',motorTorqueScale);
fprintf('校核使用的有效保持扭矩：           %.4f N*m\n\n',effectiveHoldingTorque);

fprintf('---------------- 静态保持校核 ----------------\n');
fprintf('起点角度：                         %.2f deg\n',betaStartDeg);
fprintf('起点静态重力矩：                   %.4f N*m\n',startStaticTorque);
fprintf('起点静态利用率：                   %.1f %%\n',100*startStaticUtilization);
fprintf('终点角度：                         %.2f deg\n',betaEndDeg);
fprintf('终点静态重力矩：                   %.4f N*m\n',endStaticTorque);
fprintf('终点安全系数后需求：               %.4f N*m\n',endStaticDesignTorque);
fprintf('终点静态利用率：                   %.1f %%\n',100*endStaticUtilization);
fprintf('本路径最大静态重力利用率：         %.1f %%\n',100*maxStaticGravityUtilization);
fprintf('无安全系数静态最大偏角：           %.2f deg\n',betaStaticLimitNoSF);
fprintf('按%.2f倍安全系数静态建议最大偏角：  %.2f deg\n\n', ...
        safetyFactor,betaStaticLimitWithSF);

fprintf('---------------- 运动过程总体结果 ----------------\n');
fprintf('本次路径最大重力扭矩：             %.4f N*m\n',maxGravityTorqueOnPath);
fprintf('运动过程峰值电机扭矩：             %.4f N*m\n',peakRequiredTorque);
fprintf('安全系数后峰值需求：               %.4f N*m\n',peakDesignTorque);
fprintf('峰值发生时间：                     %.4f s\n',t(idxPeak));
fprintf('峰值发生角度：                     %.2f deg\n',beta(idxPeak)*180/pi);
fprintf('峰值发生转速：                     %.4f rpm\n',speedRPM(idxPeak));
fprintf('运动过程RMS电机扭矩：              %.4f N*m\n',rmsRequiredTorque);
fprintf('运动过程最高转速：                 %.4f rpm\n',maxSpeedRPM);
fprintf('最高转速发生时间：                 %.4f s\n',t(idxMaxSpeed));
fprintf('运动过程最大角加速度：             %.4f rad/s^2\n\n',maxAcceleration);

fprintf('---------------- 全过程最危险点 ----------------\n');
fprintf('最危险点时间：                     %.4f s\n',t(idxWorst));
fprintf('最危险点角度：                     %.2f deg\n',beta(idxWorst)*180/pi);
fprintf('最危险点转速：                     %.4f rpm\n',speedRPM(idxWorst));
fprintf('惯性项 J*alpha：                   %+.6f N*m\n',torqueInertia(idxWorst));
fprintf('重力作用于负载的扭矩：             %+.6f N*m\n',torqueGravityOnLoad(idxWorst));
fprintf('电机静态抗重力扭矩：               %+.6f N*m\n',torqueMotorForStaticBalance(idxWorst));
fprintf('摩擦项：                           %+.6f N*m\n',torqueFriction(idxWorst));
fprintf('动态及摩擦附加项：                 %+.6f N*m\n',torqueDynamicAndFriction(idxWorst));
fprintf('电机总需求扭矩：                   %+.6f N*m\n',torqueMotorRequired(idxWorst));
fprintf('电机总需求扭矩绝对值：             %.6f N*m\n',torqueRequiredAbs(idxWorst));
fprintf('安全系数后需求：                   %.6f N*m\n',torqueDesignDemand(idxWorst));
fprintf('曲线可用扭矩：                     %.6f N*m\n',torqueMotorAvailable(idxWorst));
fprintf('最大总扭矩利用率：                 %.1f %%\n',100*maxUtilization);
fprintf('最小扭矩余量：                     %.6f N*m\n\n',minimumTorqueMargin);

fprintf('---------------- 仅运动阶段最危险点 ----------------\n');
fprintf('时间：                             %.4f s\n',t(idxMovingWorst));
fprintf('角度：                             %.2f deg\n',beta(idxMovingWorst)*180/pi);
fprintf('转速：                             %.4f rpm\n',speedRPM(idxMovingWorst));
fprintf('运动阶段最大利用率：               %.1f %%\n\n',100*maxMovingUtilization);

if any(overloadMask)
    fprintf('---------------- 超过录入曲线的区间 ----------------\n');
    fprintf('首次超过100%%时间：                 %.4f s\n',t(idxFirstOverload));
    fprintf('最后超过100%%时间：                 %.4f s\n',t(idxLastOverload));
    fprintf('按采样点累计超过100%%时长：         %.4f s\n\n',overloadDuration);
end

fprintf('---------------- 判定 ----------------\n');
if passMotorCurve
    fprintf('动态曲线校核：【通过】\n');
else
    fprintf('动态曲线校核：【不通过】\n');
end

if passEndStatic
    fprintf('终点静态保持校核：【通过】\n');
else
    fprintf('终点静态保持校核：【不通过】\n');
end

if maxUtilization > 1
    fprintf(['\n解释：最大利用率超过100%%，表示按照当前质量、质心距离、轨迹以及' ...
             '录入的扭矩曲线，理论需求高于曲线值。\n']);
    fprintf(['这不等于电机实际电流已达到相同比例，也不一定立即失步；' ...
             '但表示按当前模型没有足够的理论扭矩余量。\n']);
end

if strcmpi(ringCenterDistanceMode,'measured')
    geometryDistance = Lrod + Rring;
    if abs(dRingCenter - geometryDistance) > 1e-6
        fprintf(['\n提示：当前采用实测圆环质心距离 %.4f m；按"连接在圆周最低点"的' ...
                 '简单几何推算值为 %.4f m。\n'],dRingCenter,geometryDistance);
    end
end

if mRod >= 0.1
    fprintf(['提示：当前连杆质量为 %.1f g。请确认没有把20 g误填为0.200 kg，' ...
             '也没有把夹具质量重复计入。\n'],1000*mRod);
end

if mTopClamp == 0
    fprintf(['提示：mTopClamp当前为0。若顶部金属夹具未包含在mRod或mRing中，' ...
             '请称重后填写。\n']);
end

if motorTorqueScale == 1
    fprintf(['提示：motorTorqueScale当前为1.00，校核完全依据录入的标称曲线。' ...
             '实际驱动条件不同，应优先用拉力计实测轴端可用扭矩进行标定。\n']);
end

if any(outsideCurve)
    fprintf(['警告：部分转速超过扭矩曲线最大转速 %.1f rpm，' ...
             '这些位置已按无可靠可用扭矩处理。\n'],curveMaxRPM);
end


%% ===================== 8. 保存结果变量和可选CSV =====================

simulationResults = struct();
simulationResults.time_s = t;
simulationResults.beta_deg = beta*180/pi;
simulationResults.speed_rpm = omega*60/(2*pi);
simulationResults.acceleration_rad_s2 = alpha;
simulationResults.torqueGravityOnLoad_Nm = torqueGravityOnLoad;
simulationResults.torqueInertia_Nm = torqueInertia;
simulationResults.torqueFriction_Nm = torqueFriction;
simulationResults.torqueMotorRequired_Nm = torqueMotorRequired;
simulationResults.torqueMotorAvailable_Nm = torqueMotorAvailable;
simulationResults.utilizationTotal_percent = 100*utilizationTotal;
simulationResults.utilizationStaticGravity_percent = ...
    100*utilizationStaticGravity;

if exportCSV
    resultTable = table( ...
        t,beta*180/pi,omega*60/(2*pi),alpha, ...
        torqueGravityOnLoad,torqueInertia,torqueFriction, ...
        torqueMotorRequired,torqueRequiredAbs, ...
        torqueMotorAvailable,100*utilizationTotal, ...
        100*utilizationStaticGravity, ...
        'VariableNames',{ ...
        'Time_s','Beta_deg','Speed_rpm','Acceleration_rad_s2', ...
        'GravityTorqueOnLoad_Nm','InertiaTorque_Nm','FrictionTorque_Nm', ...
        'MotorRequiredTorque_Nm','MotorRequiredTorqueAbs_Nm', ...
        'MotorAvailableTorque_Nm','TotalUtilization_percent', ...
        'StaticGravityUtilization_percent'});
    writetable(resultTable,csvFileName);
    fprintf('\n已导出CSV：%s\n',csvFileName);
end


%% ===================== 9. 绘图 =====================

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
title('电机转速（带方向）');

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
plot(t, torqueMotorAvailable/safetyFactor,'--','LineWidth',1.5);
plot(t,-torqueMotorAvailable/safetyFactor,'--','LineWidth',1.5);
grid on;
xlabel('时间 / s');
ylabel('扭矩 / N*m');
title(sprintf('电机需求扭矩与允许范围（安全系数 %.2f）',safetyFactor));
legend('惯性项 J\alpha', ...
       '电机抗重力项 -T_g', ...
       '摩擦项', ...
       '电机总需求扭矩', ...
       '允许正扭矩', ...
       '允许负扭矩', ...
       'Location','best');


figure('Name','静态与动态利用率','Color','w');
plot(t,100*utilizationTotal,'LineWidth',1.6);
hold on;
plot(t,100*utilizationStaticGravity,'LineWidth',1.3);
plot([t(1),t(end)],[100,100],'--','LineWidth',1.2);
grid on;
xlabel('时间 / s');
ylabel('扭矩利用率 / %');
title('总扭矩利用率与静态重力利用率');
legend('动态总利用率', ...
       '仅静态重力利用率', ...
       '100%极限', ...
       'Location','best');


figure('Name','电机扭矩-转速校核','Color','w');
plot(motorCurveRPM,effectiveMotorCurveNm,'o-','LineWidth',1.5);
hold on;
scatter(speedRPM,torqueDesignDemand,10,'filled');
plot(speedRPM(idxWorst),torqueDesignDemand(idxWorst),'kp', ...
    'MarkerSize',12,'MarkerFaceColor','y');
grid on;
xlabel('电机转速 / rpm');
ylabel('扭矩 / N*m');
title('安全系数后需求工作点与电机扭矩-转速曲线');
legend('标定后的电机可用扭矩曲线', ...
       '安全系数后需求工作点', ...
       '最危险工作点', ...
       'Location','best');
xlim([0,max([curveMaxRPM,maxSpeedRPM])*1.05]);
ylim([0,max([max(effectiveMotorCurveNm),max(torqueDesignDemand)])*1.15]);


figure('Name','最危险点扭矩分解','Color','w');
componentValues = [ ...
    torqueMotorForStaticBalance(idxWorst), ...
    torqueInertia(idxWorst), ...
    torqueFriction(idxWorst), ...
    torqueMotorRequired(idxWorst)];
bar(componentValues);
grid on;
set(gca,'XTick',1:4);
set(gca,'XTickLabel',{ ...
    '抗重力','惯性','摩擦','总需求'});
ylabel('扭矩 / N*m');
title(sprintf('最危险点扭矩分解：t=%.3f s，\beta=%.2f°', ...
      t(idxWorst),beta(idxWorst)*180/pi));


%% ===================== 10. 可选简化动画 =====================

if enableAnimation
    figure('Name','机构简化动画','Color','w');

    totalReach = max([Lrod,dRingCenter + Rring])*1.2;

    for k = 1:animationStep:N
        clf;

        b = beta(k);

        % 连杆方向：beta=0时竖直向上。
        rodEndX = Lrod*sin(b);
        rodEndY = Lrod*cos(b);

        ringCenterX = dRingCenter*sin(b);
        ringCenterY = dRingCenter*cos(b);

        % 在机构转动平面内画圆环。
        phi = linspace(0,2*pi,200);
        ringXlocal = Rring*cos(phi);
        ringYlocal = Rring*sin(phi);

        % 圆环随连杆整体转动。
        ringX = ringCenterX + ringXlocal*cos(b) + ringYlocal*sin(b);
        ringY = ringCenterY - ringXlocal*sin(b) + ringYlocal*cos(b);

        plot([0,rodEndX],[0,rodEndY],'-','LineWidth',3);
        hold on;
        plot(ringX,ringY,'-','LineWidth',1.5);
        plot(0,0,'ko','MarkerFaceColor','k');

        axis equal;
        axis(totalReach*[-1,1,-1,1]);
        grid on;
        xlabel('水平位置 / m');
        ylabel('竖直位置 / m');
        title(sprintf('t = %.3f s,  \\beta = %.2f deg', ...
              t(k),beta(k)*180/pi));
        drawnow;
    end
end
