function VehAccLimit = VehSpd_to_VehAccLimit(VehSpd, A, B, C, MaxEngTrq, gear_ratio, FD, WhlRad, M)
% Refer E:\Ecocar\Fuel_map_approximation\STFm_to_VAFm\Flowchart_STFm_to_VAFm.drawio for
% explanation
F_resistance = A + B*VehSpd + C*VehSpd^2;
T_trac = gear_ratio * FD * MaxEngTrq;
F_trac = T_trac/WhlRad;
VehAccLimit = (F_trac-F_resistance)/M;
end