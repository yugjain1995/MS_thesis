% Refer E:\Ecocar\Fuel_map_approximation\STFm_to_VAFm\Flowchart_STFm_to_VAFm.drawio for
% explanation
clear;clc;close all;
%% Setting up parameters



gear = 6;
load('GM_9T50_M3DandH_LTG_and_Final_Drive_powertrain_data.mat');
v_max =  transmission_data.gear_velocity_range(gear,2);
v_min =  transmission_data.gear_velocity_range(gear,1);
FD = transmission_data.final_drive;
gear_ratio = transmission_data.forward_gear_ratios(gear);

load('GM_Engine_LTG_2.0LT_I4.mat');
eng_spd = engine_data.eng_spd * 2 * pi/60; % rad/sec
eng_trq = engine_data.eng_trq;

% Velocity range for map
VehSpd = v_min:(v_max-v_min)/(length(eng_spd)-1):v_max;

% Acceleration range for map
VehAccReq = 0:2/(length(eng_spd)-1):2;

% Finding maximum engine trq curve
EngSpd = engine_data.EngSpd_EngTrq_Fuel_rate_Table.EngineSpeed;
EngTrq = engine_data.EngSpd_EngTrq_Fuel_rate_Table.ObservedBrakeTorque;
STFm = engine_data.fuel_rate_map;
MaxEngTrqCurve = [];
i = 1; j = 1;
while(1)
    j = j + 1;
    if(j < length(EngSpd))
        if (EngSpd(j) == EngSpd(i))
            continue;
        else
            MaxEngTrqCurve = [MaxEngTrqCurve; [EngSpd(j-1) * 2*pi/60 EngTrq(j-1)]];
            i = j;
            continue;
        end
    else
        MaxEngTrqCurve = [MaxEngTrqCurve; [EngSpd(j-1) * 2*pi/60 EngTrq(j-1)]];
        break;
    end
end

load('GMBlazerNominalParameters.mat');
Mass = vehicle.Mass;
A = vehicle.RollingResistanceCoefficient;
B = vehicle.DrivelineResistanceCoefficient;
C = vehicle.AerodynamicDragCoefficient;
WhlRad = vehicle.WheelRadius;
%% Generating map
VAFm = zeros(size(STFm));
for i = 1:size(STFm,2)
    for j = 1:size(STFm,1)
        MaxEngTrq = VehSpd_to_MaxEngTrq(VehSpd(i), WhlRad, gear_ratio, FD, MaxEngTrqCurve);
        
        VehAccLimit = VehSpd_to_VehAccLimit(VehSpd(i), A, B, C, MaxEngTrq, gear_ratio, FD, WhlRad, Mass);
        
        if (VehAccReq(j) <= VehAccLimit)
            EngTrqReq = function2(VehSpd(i), A, B, C, VehAccReq(j), Mass, WhlRad, gear_ratio, FD);
        else
            EngTrqReq = NaN;
            VAFm(i,j) = NaN;
            continue;
        end
        
        EngSpd_at_VehSpd = function1(VehSpd(i), WhlRad, gear_ratio, FD);
        
        VAFm(i,j) = interp2(eng_trq, eng_spd, STFm, EngTrqReq, EngSpd_at_VehSpd);
    end
end

surf(eng_trq,eng_spd,STFm);
xlabel('eng_trq');
ylabel('eng_spd');
zlabel('fuel rate');
title('STFm');
figure;

surf(VehAccReq,VehSpd,VAFm);
xlabel('VehAccReq');
ylabel('VehSpd');
zlabel('fuel rate');
title('VAFm');


