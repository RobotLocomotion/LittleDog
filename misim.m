function misim

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('LittleDog.urdf',.01,options);
p = p.removeCollisionGroupsExcept({'feet'});
p = p.compile();
warning(w);

x0 = Point(getStateFrame(p));
hip_pitch = 1;
knee = 1.55;
x0.front_right_hip_pitch = hip_pitch;
x0.front_right_knee = -knee;
x0.front_left_hip_pitch = hip_pitch;
x0.front_left_knee = -knee;
x0.back_right_hip_pitch = -hip_pitch;
x0.back_right_knee = knee;
x0.back_left_hip_pitch = -hip_pitch;
x0.back_left_knee = knee;
x0.base_z = 2*0.146;

v = p.constructVisualizer(struct('viewer','BotVisualizer'));
v.drawWrapper(0,x0);

% construct PD control
Kp = 2*eye(8);
Kd = diag([0.5; 0.16; 0.5; 0.16; 0.5; 0.16; 0.5; 0.16]);
Kp = .8*Kp;  Kd = .1*Kd;
qa0 = x0(getActuatedJoints(p)); 
%% y = Kp*(qa0-qa) - Kd*va;
D = zeros(getNumInputs(p),getNumStates(p));
D(:,getActuatedJoints(p)) = -Kp;
D(:,getNumPositions(p)+getActuatedJoints(p)) = -Kd;
pd = AffineSystem([],[],[],[],[],[],[],D,Kp*qa0);

dt = 0.005; %0.01;
xtraj = misim(getManipulator(p),x0,dt,ceil(5/dt),1,pd,v);

%v.playback(xtraj);

