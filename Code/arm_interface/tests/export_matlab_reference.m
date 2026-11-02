% Run with MATLAB to regenerate independent Python-port regression fixtures.
% Does not change the MATLAB sources or their results folders.
addpath('C:/Users/hrish/Desktop/new files/MATLAB');
robot = RRR_model();
rng(23);
angles = [0 0 0; 20 30 -50; -60 75 45; 90 180 -90; -90 0 90; ...
          0 90 0.02; 180 45 60; ...
          [-90 0 -90]+180*rand(30,3)];
cases = struct([]);
for k = 1:size(angles,1)
    q = angles(k,:);
    [T, points, J] = RRR_FK(q,robot);
    target = T(1:3,4)';
    options = struct('jointLimits_deg',[-90 90;0 180;-90 90], ...
                     'reference_deg',q);
    report = RRR_IK_solve(target,robot,options);
    valid = report.Solutions.ValidPosition & ~report.Solutions.Singular & ...
            ~report.Solutions.NearSingular;
    cases(k).q = q;
    cases(k).servo = RRR_servo_map(q,'dhToServo');
    cases(k).T = T;
    cases(k).points = points;
    cases(k).J = J;
    cases(k).target = target;
    cases(k).app_target = RRR_cartesian_frame(target);
    cases(k).status = char(report.Status);
    cases(k).safe_solutions = report.Solutions{valid,{'Theta1_deg','Theta2_deg','Theta3_deg'}};
end
[~,calibration] = RRR_servo_map([0 0 0],'dhToServo');
[~,axis_signs] = RRR_cartesian_frame([0 0 0]);
fixture = struct('source','MATLAB R2025b: RRR_model.m, RRR_FK.m, RRR_IK_solve.m, RRR_servo_map.m', ...
                 'a',robot.a,'d',robot.d,'alpha',robot.alpha_deg, ...
                 'servo_offsets',calibration.Offsets_deg,'servo_directions',calibration.Directions, ...
                 'cartesian_axis_signs',axis_signs,'cases',cases);
destination = fullfile(fileparts(mfilename('fullpath')),'matlab_reference.json');
fid = fopen(destination,'w');
assert(fid>=0,'Cannot write fixture');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s',jsonencode(fixture,PrettyPrint=true));
fprintf('Exported %d independent FK/Jacobian/IK cases to %s\n',numel(cases),destination);
