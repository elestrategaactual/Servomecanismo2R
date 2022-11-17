% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(7).translation = [0.0 0.0 0.0];
smiData.RigidTransform(7).angle = 0.0;
smiData.RigidTransform(7).axis = [0.0 0.0 0.0];
smiData.RigidTransform(7).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-2.7046599100789566 7.9562608438217497 -1.282002317319517];  % in
smiData.RigidTransform(1).angle = 2.5550486249257842;  % rad
smiData.RigidTransform(1).axis = [-0.66734550022494143 -0.29463111686405469 -0.68399012295863049];
smiData.RigidTransform(1).ID = "B[sim_MARCADOR:1:-:sim_LINK2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-2.7046599100788997 7.9562608438212683 -1.2820023173194548];  % in
smiData.RigidTransform(2).angle = 2.5550486249257811;  % rad
smiData.RigidTransform(2).axis = [-0.66734550022494044 -0.29463111686405546 -0.68399012295863115];
smiData.RigidTransform(2).ID = "F[sim_MARCADOR:1:-:sim_LINK2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-1.0539155298220759 0.84006950099788147 0.49450154930973977];  % in
smiData.RigidTransform(3).angle = 0.036552128569826516;  % rad
smiData.RigidTransform(3).axis = [1.6382437013613327e-14 -1 -2.9943981182899154e-16];
smiData.RigidTransform(3).ID = "B[sim_LINK1:1:-:sim_BASE:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-0.30963910088960461 0.077762118509034472 -1.8523399759230592];  % in
smiData.RigidTransform(4).angle = 0.036552128569825218;  % rad
smiData.RigidTransform(4).axis = [3.3239727273999159e-16 -1 -6.075590384936046e-18];
smiData.RigidTransform(4).ID = "F[sim_LINK1:1:-:sim_BASE:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-6.8647544363917419 2.7124439575547417 -2.6160221963580348];  % in
smiData.RigidTransform(5).angle = 0.036552128569828604;  % rad
smiData.RigidTransform(5).axis = [2.7802657598463769e-14 -1 -5.0817973862572034e-16];
smiData.RigidTransform(5).ID = "B[sim_LINK2:1:-:sim_LINK1:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-4.7119698688904386 7.0116316627653195 -0.15142082170559618];  % in
smiData.RigidTransform(6).angle = 0.036552128569827473;  % rad
smiData.RigidTransform(6).axis = [3.9543404124887278e-14 -1 -7.2277827043649892e-16];
smiData.RigidTransform(6).ID = "F[sim_LINK2:1:-:sim_LINK1:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [3.8671123106586793 1.1548333574908765 6.1115813706496009];  % in
smiData.RigidTransform(7).angle = 0.038100763323246495;  % rad
smiData.RigidTransform(7).axis = [0 1 0];
smiData.RigidTransform(7).ID = "RootGround[sim_BASE:1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.15885979394606597;  % lbm
smiData.Solid(1).CoM = [-3.2695897596127645 4.5829206458419414 0.072077325701843234];  % in
smiData.Solid(1).MoI = [0.99980482580070573 0.37404405955725489 1.3439994181545705];  % lbm*in^2
smiData.Solid(1).PoI = [0.059942941261572083 -0.035887699194623125 0.57430767839019115];  % lbm*in^2
smiData.Solid(1).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "sim_LINK1.ipt_{9B5B08C2-4DBF-8609-13C2-DCA04C8930DE}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.05724681977525977;  % lbm
smiData.Solid(2).CoM = [-2.5418112338829442 8.3510593493957437 -0.71739790832176609];  % in
smiData.Solid(2).MoI = [0.070444680926250239 0.071547007319145928 0.0083452461247550975];  % lbm*in^2
smiData.Solid(2).PoI = [-0.010508783173111364 0.013520293505078114 0.0021118404779974132];  % lbm*in^2
smiData.Solid(2).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "sim_MARCADOR.ipt_{C1B86DB5-4478-05CE-3A7F-2D946902D73E}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.33807098703407529;  % lbm
smiData.Solid(3).CoM = [-0.79830671054796865 -0.47359383363102836 -3.1663393739160681];  % in
smiData.Solid(3).MoI = [1.9192458631527589 2.172525291307307 0.46749970242615674];  % lbm*in^2
smiData.Solid(3).PoI = [0.0076586831331184159 0.072256568742283173 -0.10439252412028047];  % lbm*in^2
smiData.Solid(3).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "sim_BASE.ipt_{AC4E670F-4606-7D18-FF88-EEB3B950D848}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.12974687168031945;  % lbm
smiData.Solid(4).CoM = [-5.0789712316100983 4.8021621859156411 -1.4035504150082534];  % in
smiData.Solid(4).MoI = [0.47509660767587608 0.36254830844943486 0.7514375669843103];  % lbm*in^2
smiData.Solid(4).PoI = [-0.051566880472791093 -0.044215100834450967 -0.35501267392348201];  % lbm*in^2
smiData.Solid(4).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "sim_LINK2.ipt_{61C53538-414D-E790-F9BA-059944D77944}";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -148.49624999818136;  % deg
smiData.RevoluteJoint(1).ID = "[sim_MARCADOR:1:-:sim_LINK2:1]";

smiData.RevoluteJoint(2).Rz.Pos = -43.6313764498436;  % deg
smiData.RevoluteJoint(2).ID = "[sim_LINK1:1:-:sim_BASE:1]";

smiData.RevoluteJoint(3).Rz.Pos = -7.0879769650839695;  % deg
smiData.RevoluteJoint(3).ID = "[sim_LINK2:1:-:sim_LINK1:1]";

