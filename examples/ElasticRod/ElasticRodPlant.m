classdef ElasticRodPlant < DrakeSystem

    properties
    end
    
    methods
      function obj = ElasticRodPlant
          obj = obj@DrakeSystem(0,3,1,3, false, true);
          obj = setInputFrame(obj,CoordinateFrame('ElasticRodInput',1,'u',{'phi'}));
          obj = setStateFrame(obj,CoordinateFrame('ElasticRodState',3,'x',{'xcoord','ycoord', 'theta'}));
          obj = setOutputFrame(obj,obj.getStateFrame);
      end
      
      function [f,df] = dynamics(obj,t,x,u)
          f = [cos(x(3)), sin(x(3)), u]';
          dfx = [[0, 0, -sin(x(3))]; [0, 0, cos(x(3))]; [0, 0, 0]];
          dfu = [0, 0, 1]';
          df = [zeros(3, 1), dfx, dfu];
      end
      
      function [utraj,xtraj]=rodTrajectory(obj)
          x0 = zeros(3,1); 
          xf = [1, 0.5, pi/2]';
          tf0 = 3;

          N = 21;
          prog = DircolTrajectoryOptimization(obj,N,[2]);
          prog = prog.addStateConstraint(ConstantConstraint(x0),1);
          prog = prog.addStateConstraint(ConstantConstraint(xf),N);
          prog = prog.addRunningCost(@cost);
          prog = prog.addFinalCost(@finalCost);

          traj_init.x = PPTrajectory([0, 0, 0]);

          [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);


          function [g,dg] = cost(dt,x,u)
            R = 1;
            g = sum((R*u).*u,1);
            dg = [zeros(1,1+size(x,1)),2*u'*R];
          end

          function [h,dh] = finalCost(t,x)
            h = 0;
            dh = [0,zeros(1,size(x,1))];
            return;
          end      
      end
    end
    
end

