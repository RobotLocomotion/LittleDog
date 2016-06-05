%function milinmpc

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
x0.base_z = 0.146;

v = p.constructVisualizer(struct('viewer','BotVisualizer'));
v.drawWrapper(0,x0);

% construct PD control
Q = eye(getNumStates(p));
R = .1*eye(getNumInputs(p));
qa0 = x0(getActuatedJoints(p)); 

dt = 0.05; %0.01;
[u0,xtraj] = milinmpc(getManipulator(p),[0.2;0;.146;qa0],x0,dt,20,Q,R,1);

v.playback(xtraj);

