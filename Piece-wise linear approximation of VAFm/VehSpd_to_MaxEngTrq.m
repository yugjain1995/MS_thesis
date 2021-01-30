function MaxEngTrq = VehSpd_to_MaxEngTrq(VehSpd, WhlRad, gear_ratio, FD, MaxEngTrqCurve)
% Refer E:\Ecocar\Fuel_map_approximation\STFm_to_VAFm\Flowchart_STFm_to_VAFm.drawio for
% explanation
WhlSpd = VehSpd/WhlRad;
EngSpd = WhlSpd*gear_ratio*FD;
MaxEngTrq = interp1(MaxEngTrqCurve(:,1),MaxEngTrqCurve(:,2),EngSpd);
MaxEngTrq(MaxEngTrq < 0) = 0;
end