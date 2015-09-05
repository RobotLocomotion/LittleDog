function [sol,v,prog,xtraj] = gaitOptimization(gait,seed)

  checkDependency('lcmgl');
  if nargin < 1, gait = 'walking_trot'; end
  if nargin < 2, seed = []; end
  if nargin < 3, options = struct(); end
  options = parseOptionsStruct(options); 

  % Construct RigidBodyManipulator
  robot = LittleDog;
  %mu = 1.16; % rubber on rubber
  mu = 1; % rubber on rubber

  % Create convenience variables
  nq = robot.getNumPositions();
  foot = struct('id',[],'in_stance',[]);
  foot(1).id = robot.findFrameId('front_left_foot_center');
  foot(2).id = robot.findFrameId('front_right_foot_center'); 
  foot(3).id = robot.findFrameId('back_left_foot_center');
  foot(4).id = robot.findFrameId('back_right_foot_center');

  % setup gait
  is_laterally_symmetric = false;  % if true, then plan half a gait and mirror
  check_self_collision = false;
  switch(gait)
    % http://upload.wikimedia.org/wikipedia/commons/thumb/c/cf/Gait_graphs.jpg/500px-Gait_graphs.jpg
    case 'running_trot'
      N = 21;
      foot(2).in_stance = 4:17;
      foot(3).in_stance = 4:17;
      if ~isfield(options,'speed'), options.speed = .9; end
      if ~isfield(options,'stride_length'), options.stride_length = .55; end
      is_laterally_symmetric = true;
    case 'walking_trot'
      N = 21;
      foot(1).in_stance = 1:11;
      foot(2).in_stance = 9:N;
      foot(3).in_stance = 9:N;
      foot(4).in_stance = 1:11;
      if ~isfield(options,'speed'), options.speed = .4; end
      if ~isfield(options,'stride_length'), options.stride_length = .25; end
      is_laterally_symmetric = true;
    case 'rotary_gallop'
      N = 41;
      foot(1).in_stance = 8:19;
      foot(2).in_stance = 4:15;
      foot(3).in_stance = 25:35;
      foot(4).in_stance = 27:37;
      if ~isfield(options,'speed'), options.speed = 1; end
      if ~isfield(options,'stride_length'), options.stride_length = .65; end
      check_self_collision = true;
    case 'bound'
      N = 41;
      foot(1).in_stance = 7:18;
      foot(2).in_stance = 7:18;
      foot(3).in_stance = 22:32;
      foot(4).in_stance = 22:32;
      if ~isfield(options,'speed'), options.speed = 1.2; end
      if ~isfield(options,'stride_length'), options.stride_length = .55; end
      check_self_collision = true;
    otherwise
      error('unknown gait. use one of "running_trot", "walking_trot", "rotary_gallop", or "bound" ');
  end
  
  
  % Construct visualization tools
  v = constructVisualizer(robot);
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'gait');

  % Load nominal data
  xstar = home(robot);
  qstar = xstar(1:nq);
  v.draw(0,qstar);
  
  q0 = qstar;
  q0(1) = 0;
  qf = q0;
  qf(1) = options.stride_length;
  com_0 = robot.getCOM(q0);
  com_f = robot.getCOM(qf);

  % Set up time limits
  T = options.stride_length/options.speed;
  if is_laterally_symmetric  
    qf(1) = qf(1)/2;
    T = T/2;
  end
  tf_range = T*[.9,1.1];
  
  % Set up cost variables
  q_nom = bsxfun(@times,qstar,ones(1,N));
  state_cost = Point(getStateFrame(robot),ones(getNumStates(robot),1));
  state_cost.base_x = 0;
  state_cost.base_y = 0;
  state_cost.base_roll = 10;
  state_cost.base_yaw = 10;
  state_cost.front_left_hip_roll = 5;
  state_cost.front_right_hip_roll = 5;
  state_cost.back_left_hip_roll = 5;
  state_cost.back_right_hip_roll = 5;
  state_cost = double(state_cost);
  Q = diag(state_cost(1:nq)); 
  Qv = diag(state_cost(nq+1:end));
  Q_comddot = diag([1,1,1]);
  Q_contact_force = 5*eye(3);

  % Set up linearized friction cone edges
  num_edges = 3;
  FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
  FC_axis = [0;0;1];
  FC_perp1 = rotx(pi/2)*FC_axis;
  FC_perp2 = cross(FC_axis,FC_perp1);
  FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
    bsxfun(@times,sin(FC_angles),FC_perp2)));
  FC_edge = robot.getMass()*norm(robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));

  for i=1:4
    contact_wrench_struct(i).active_knot = foot(i).in_stance;
    contact_wrench_struct(i).cw = LinearFrictionConeWrench(robot,foot(i).id,zeros(3,1),FC_edge);
  end
  
  options.time_option = 2;
  prog = ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options);
  if options.visualize
    prog = prog.addDisplayFunction(@(w)displayCallback(N,w),[prog.h_inds(:);reshape(prog.com_inds(3,:),[],1);prog.q_inds(:)]);
  end
  
  % Kinematic constraints
  lb = [NaN; NaN; 0];
  ub = [NaN; NaN; 0];
  for i=1:4
    in_swing = setdiff(1:N,foot(i).in_stance);
    if ~isempty(foot(i).in_stance)
      % stance feet on the ground (only constrains z)
      prog = prog.addRigidBodyConstraint(WorldPositionConstraint(robot,foot(i).id,zeros(3,1),lb,ub),foot(i).in_stance);

      % stance feet stationary constraints
      prog = prog.addRigidBodyConstraint(WorldFixedPositionConstraint(robot,foot(i).id,zeros(3,1)),{foot(i).in_stance});
    end
    min_clearance = 0.01;
    if ~isempty(in_swing)
      % swing foot clearance 
      prog = prog.addRigidBodyConstraint(WorldPositionConstraint(robot,foot(i).id,zeros(3,1),lb+[NaN;NaN;min_clearance],NaN(size(lb))),in_swing);
    end
  end
  
  % com height constraint
  lb = .125; ub = nan; %.15;
  prog = prog.addConstraint(BoundingBoxConstraint(lb*ones(N,1),ub*ones(N,1)),prog.com_inds(3,1:N));

  % comdot in x always positive 
  prog = prog.addConstraint(BoundingBoxConstraint(zeros(N,1),nan(N,1)),prog.comdot_inds(1,1:N));
  
  % zero roll pitch and yaw constraint
  prog = prog.addBoundingBoxConstraint(ConstantConstraint(zeros(3*N,1)),prog.q_inds([4:6],1:N));
  
  % Add Timestep bounds
  h_min = 1/(2*N)*T; h_max = 2/N*T;
  prog = prog.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(N-1,1),h_max*ones(N-1,1)),prog.h_inds(:));
  
  % Add initial condition constraints
  prog = prog.addConstraint(ConstantConstraint([0;0]),prog.com_inds(1:2,1));
  prog = prog.addConstraint(ConstantConstraint(0),prog.comdot_inds(3,1));

  % Add final condition constraints
  if is_laterally_symmetric
    prog = prog.addConstraint(ConstantConstraint(options.stride_length/2), prog.com_inds(1,end));
  else
    prog = prog.addConstraint(ConstantConstraint(options.stride_length), prog.com_inds(1,end));
  end
  
  % Add collision constraints
  if check_self_collision
    min_distance = 0.003;
    active_collision_options.collision_groups = {'right_lower_legs','left_lower_legs'};
    prog = prog.addRigidBodyConstraint(MinDistanceConstraint(robot,min_distance,[-inf,inf],active_collision_options),1:N);
  end
  
  % Add periodicity constraints
  if is_laterally_symmetric
    % for the joints
    prog = prog.addConstraint(halfPeriodicConstraint(robot),prog.q_inds(:,[1,end]));
    % for the floating base velocity
    prog = prog.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]),prog.v_inds(1:3,[1,end]));
    % for the center of mass velocity
    prog = prog.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]),prog.comdot_inds(1:3,[1,end]));
  else
    % everything except base_x is periodic
    prog = prog.addConstraint(LinearConstraint(zeros(nq-1,1),zeros(nq-1,1),[eye(nq-1),-eye(nq-1)]),prog.q_inds(2:end,[1,end]));

    nv = getNumVelocities(robot);
    prog = prog.addConstraint(LinearConstraint(zeros(nv,1),zeros(nv,1),[eye(nv),-eye(nv)]),prog.v_inds(:,[1,end]));
    
    prog = prog.addConstraint(LinearConstraint(zeros(5,1),zeros(5,1),[eye(5),-eye(5)]),[prog.com_inds(2:3,[1,end]); prog.comdot_inds(1:3,[1,end])]);
  end
  
  if isempty(seed)
    x_seed = zeros(prog.num_vars,1);
    q_seed = linspacevec(q0,qf,N);
    v_seed = gradient(q_seed);
    com_seed = linspacevec(com_0,com_f,N);
    comdot_seed = gradient(com_seed);
    comddot_seed = gradient(comdot_seed);
    lambda_seed = sqrt(2)/24;
    x_seed(prog.h_inds) = T/N;
    x_seed(prog.q_inds(:)) = reshape(q_seed,[],1);
    x_seed(prog.v_inds(:)) = reshape(v_seed,[],1);
    x_seed(prog.com_inds(:)) = reshape(com_seed,[],1);
    x_seed(prog.comdot_inds(:)) = reshape(comdot_seed,[],1);
    x_seed(prog.comddot_inds(:)) = reshape(comddot_seed,[],1);
    x_seed(prog.lambda_inds{1}(:)) = lambda_seed;
  else
    x_seed = seed.x_sol;
  end
  
  % Set up solver options
  prog = prog.setSolverOptions('snopt','iterationslimit',1e6);
  prog = prog.setSolverOptions('snopt','majoriterationslimit',options.major_iteration_limit);
  prog = prog.setSolverOptions('snopt','majorfeasibilitytolerance',5e-6);
  prog = prog.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
  prog = prog.setSolverOptions('snopt','superbasicslimit',2000);
  prog = prog.setSolverOptions('snopt','linesearchtolerance',0.9);
%  prog = prog.setSolverOptions('snopt','print',sprintf('snopt_%s.out',options.suffix));
  
  % Solve trajectory optimization
  tic
  %profile on;
  [x_sol,~,~] = prog.solve(x_seed);
  %profile off;
  toc
  
  % Parse trajectory optimization output
  sol.x_sol = x_sol;
  sol.q = reshape(x_sol(prog.q_inds(:)),nq,N);
  sol.v = reshape(x_sol(prog.v_inds(:)),nq,N);
  sol.h = reshape(x_sol(prog.h_inds),1,[]);
  sol.t = cumsum([0 sol.h]);
  sol.com = reshape(x_sol(prog.com_inds),3,[]);
  sol.comdot = reshape(x_sol(prog.comdot_inds),3,[]);
  sol.comddot = reshape(x_sol(prog.comddot_inds),3,[]);
  sol.H = reshape(x_sol(prog.H_inds),3,[]);
  sol.Hdot = reshape(x_sol(prog.Hdot_inds),3,[])*prog.torque_multiplier;
  sol.lambda = cell(2,1);
  for i = 1:numel(prog.lambda_inds)
    sol.lambda{i} = reshape(x_sol(prog.lambda_inds{i}),size(prog.lambda_inds{i},1),[],N);
  end
  sol.xtraj= PPTrajectory(foh(sol.t,[sol.q;sol.v]));
  sol.xtraj= sol.xtraj.setOutputFrame(robot.getStateFrame);

  % Save results
%  save(sprintf('results_%s',options.suffix),'sol');
  
  if is_laterally_symmetric
    xtraj = halfStrideToFullStride(robot,@mirrorPositions,sol.xtraj);
  else
    xtraj = sol.xtraj;
  end
  xtraj = oneStrideToMultipleStrides(robot,xtraj,10);
  
  v.playback(xtraj,struct('slider',true))
end

function half_periodic_constraint = halfPeriodicConstraint(robot)
  num_symmetry = 12;
  num_equal = 5;
  nq = robot.getNumPositions();
  
  symmetric_matrix = zeros(num_symmetry,2*nq);
  equal_matrix = zeros(num_equal,2*nq);
  initial_indices = 1:nq;
  final_indices   = nq+(1:nq);

  function sym_mat = addSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 -1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 -1];
  end

  function sym_mat = addAntiSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 1];
  end

  function eq_mat = addEquality(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 -1];
  end

  function eq_mat = addOpposite(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 1];
  end

  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,1:2,...
    robot.getBody(robot.findJointId('front_left_hip_roll')).position_num, ...
    robot.getBody(robot.findJointId('front_right_hip_roll')).position_num);

  symmetric_matrix = addSymmetricPair(symmetric_matrix,3:4, ...
    robot.getBody(robot.findJointId('front_left_hip_pitch')).position_num, ...
    robot.getBody(robot.findJointId('front_right_hip_pitch')).position_num);
  
  symmetric_matrix = addSymmetricPair(symmetric_matrix,5:6,...
    robot.getBody(robot.findJointId('front_left_knee')).position_num,...
    robot.getBody(robot.findJointId('front_right_knee')).position_num);

  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,7:8,...
    robot.getBody(robot.findJointId('back_left_hip_roll')).position_num,...
    robot.getBody(robot.findJointId('back_right_hip_roll')).position_num);

  symmetric_matrix = addSymmetricPair(symmetric_matrix,9:10,...
    robot.getBody(robot.findJointId('back_left_hip_pitch')).position_num,...
    robot.getBody(robot.findJointId('back_right_hip_pitch')).position_num);

  symmetric_matrix = addSymmetricPair(symmetric_matrix,11:12,...
    robot.getBody(robot.findJointId('back_left_knee')).position_num,...
    robot.getBody(robot.findJointId('back_right_knee')).position_num);
  
  base_y = findPositionIndices(robot,'base_y'); base_y = base_y(1);
  equal_matrix = addOpposite(equal_matrix,1,base_y);

  base_z = findPositionIndices(robot,'base_z');
  equal_matrix = addEquality(equal_matrix,2,base_z);

  base_roll = findPositionIndices(robot,'base_roll');
  equal_matrix = addOpposite(equal_matrix,3,base_roll);

  base_pitch = findPositionIndices(robot,'base_pitch');
  equal_matrix = addEquality(equal_matrix,4,base_pitch);

  base_yaw = findPositionIndices(robot,'base_yaw');
  equal_matrix = addOpposite(equal_matrix,5,base_yaw);

  lb = zeros(num_symmetry+num_equal,1);
  ub = lb;
  half_periodic_constraint = LinearConstraint(lb,ub,[symmetric_matrix;equal_matrix]);
end

function q_mirror = mirrorPositions(robot,q)
  q_mirror = q;

  function n = position_num(joint_name)
    n = robot.getBody(robot.findJointId(joint_name)).position_num;
  end
  
  % y,roll,yaw
  q_mirror([2,4,6],:) = -q([2,4,6],:);
  
  q_mirror(position_num('front_left_hip_roll'),:) = -q(position_num('front_right_hip_roll'),:);
  q_mirror(position_num('front_right_hip_roll'),:) = -q(position_num('front_left_hip_roll'),:);
  q_mirror(position_num('back_left_hip_roll'),:) = -q(position_num('back_right_hip_roll'),:);
  q_mirror(position_num('back_right_hip_roll'),:) = -q(position_num('back_left_hip_roll'),:);
  
  q_mirror(position_num('front_left_hip_pitch'),:) = q(position_num('front_right_hip_pitch'),:);
  q_mirror(position_num('front_right_hip_pitch'),:) = q(position_num('front_left_hip_pitch'),:);
  q_mirror(position_num('back_left_hip_pitch'),:) = q(position_num('back_right_hip_pitch'),:);
  q_mirror(position_num('back_right_hip_pitch'),:) = q(position_num('back_left_hip_pitch'),:);
  
  q_mirror(position_num('front_left_knee'),:) = q(position_num('front_right_knee'),:);
  q_mirror(position_num('front_right_knee'),:) = q(position_num('front_left_knee'),:);
  q_mirror(position_num('back_left_knee'),:) = q(position_num('back_right_knee'),:);
  q_mirror(position_num('back_right_knee'),:) = q(position_num('back_left_knee'),:);
end

function displayCallback(N,x)
  h = x(1:N-1);
  ts = [0;cumsum(h)];
  com_z = x(N-1+(1:N));
  sfigure(7); 
  plot(ts,com_z,'bo-'); 
  drawnow;
end


function options = parseOptionsStruct(options_in)
  options = defaultOptionsStruct();
  for fieldname_cell = fields(options_in)'
    fieldname = fieldname_cell{1};
    if isfield(options,fieldname)
      options.(fieldname) = options_in.(fieldname);
    end
  end
end


function xtraj = halfStrideToFullStride(robot,mirror_fun,xtraj_half)
  % @param mirror_fun   -- function handle that takes an nq x N array of joint
  % positions and returns an appropriately mirrored version. See
  % mirrorAtlasPositions for an example.
  nq = robot.getNumPositions();
  t_half = xtraj_half.getBreaks();
  x_half = xtraj_half.eval(t_half);
  q_half = x_half(1:nq,:);
  v_half = x_half(nq+1:end,:);
  q_mirror = mirror_fun(robot,q_half);
  v_mirror = mirror_fun(robot,v_half);
  q_mirror(1,:) = q_mirror(1,:) + (q_half(1,end) - q_half(1,1));
  q = [q_half, q_mirror(:,2:end)];
  v = [v_half, v_mirror(:,2:end)];
  t = [t_half, t_half(2:end) + t_half(end)];
  xtraj = PPTrajectory(foh(t,[q;v]));
  xtraj = xtraj.setOutputFrame(xtraj_half.getOutputFrame());
end

function xtraj = oneStrideToMultipleStrides(robot,xtraj_stride,n_strides)
  nq = robot.getNumPositions();
  t_stride = xtraj_stride.getBreaks();
  x_stride = xtraj_stride.eval(t_stride);
  q_stride = x_stride(1:nq,:);
  v_stride = x_stride(nq+1:end,:);
  q = q_stride;
  t = t_stride;
  for i = 1:n_strides-1
    q_next = q_stride(:,2:end);
    q_next(1,:) = q_next(1,:) + (q(1,end) - q(1,1));
    q = [q, q_next];
    t = [t, t_stride(2:end) + t(end)];
  end
  v = [v_stride, repmat(v_stride(:,2:end),1,n_strides-1)];
  xtraj = PPTrajectory(foh(t,[q;v]));
  xtraj = xtraj.setOutputFrame(xtraj_stride.getOutputFrame());
end



function options = defaultOptionsStruct()
  options.visualize = true;
  options.major_iteration_limit = 200;
  options.suffix = '';
end
