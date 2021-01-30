function EngTrqReq = function2(VehSpd, A, B, C, VehAccReq, Mass, WhlRad, gear_ratio, FD)
% Refer E:\Ecocar\Fuel_map_approximation\STFm_to_VAFm\Flowchart_STFm_to_VAFm.drawio for
% explanation
F_tracReq = A + B*VehSpd + C*VehSpd.^2 + VehAccReq * Mass;
WhlTrqReq = F_tracReq*WhlRad;
EngTrqReq = WhlTrqReq/(gear_ratio * FD);
EngTrqReq(EngTrqReq < 0) = 0;
end