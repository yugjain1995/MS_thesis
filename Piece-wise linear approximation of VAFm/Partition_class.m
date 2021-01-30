classdef Partition_class
    properties
        P_ind (1,2)
        P_val (1,3)
        P_set = struct('ind', 'val')
        P_set_pre = struct('ind', 'val') % Previous partition
        C (1,3) % Fit coefficients
        rms_error (1,1)% Fit rms error
    end
    methods
        function PC = Partition_class()
            PC.P_set.ind = [];
            PC.P_set.val = [];
            PC.P_set_pre.ind = [];
            PC.P_set_pre.val = [];
        end
        
        function d = eucledian_dist(PC,point)
            arguments
                PC
                point (1,3)
            end
            d = pdist([PC.P_val;point],'euclidean');
        end
        
        function [PC] = partition_fit(PC)
            arguments
                PC
            end
            [temp,error] = lsqlin([PC.P_set.val(:,1:2),ones(size(PC.P_set.val,1),1)],PC.P_set.val(:,3),-[PC.P_set.val(:,1:2),ones(size(PC.P_set.val,1),1)],zeros(size(PC.P_set.val,1),1));
            PC.rms_error = sqrt(error/size(PC.P_set,1));
            PC.C = temp';
        end
        
        function val = affine_eval(PC,point)
            arguments
                PC
                point (1,2)
            end
            val = PC.C*[point 1]';
        end
        
        function [PC] = replace_set(PC)
            arguments
                PC
            end
            PC.P_set_pre = PC.P_set;
            PC.P_set.ind = [];
            PC.P_set.val = [];
        end
        
%         function [PC] = test(PC)
%             arguments
%                 PC
%             end
%             PC.C = 4;
%         end
    end
end