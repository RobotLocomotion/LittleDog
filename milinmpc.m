%function milinmpc

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('PlanarLittleDog.urdf',.01,options);
p = p.removeCollisionGroupsExcept({'feet'});
p = p.compile();
warning(w);

x0 = Point(getStateFrame(p));
hip_pitch = 1;
knee = 1.55;
x0.front_left_hip_pitch = hip_pitch;
x0.front_left_knee = -knee;
x0.back_left_hip_pitch = -hip_pitch;
x0.back_left_knee = knee;
x0.base_z = 0.13;

%x0.base_x = .1;
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

% construct PD control
Q = eye(getNumStates(p));
R = 10*eye(getNumInputs(p));
qa0 = x0(getActuatedJoints(p)); 

dt = 0.05; %0.01;
%x0 = double(x0) + 0.3*randn(size(x0));
v.drawWrapper(0,x0);
[x0,u0,z0] = findFixedPoint(p,double(x0));z0
v.drawWrapper(0,x0);
[u0,xtraj] = milinmpc(getManipulator(p),x0(1:getNumPositions(p)),double(u0),x0,dt,10,Q,R,inf);
figure(1); plot(u0');
v.playback(xtraj);

