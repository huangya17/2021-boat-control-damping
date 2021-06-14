% 3DOF autonomous boat and simple control logic to follow waypoint 
% Ya Huang, 2017-04-12, School of Engineering, University of Portsmouth, England
% This script calls and runs simulation file *.slx 
% 2019-10-21 YH: sdf() 3rd party function; multiple waypoints
% 2020-03-26 YH: Modified to use planar joint to rectify local and global z-axis rotation using Max's AutoBoat.slx
% 2020-04-09 YH: Figure 10 quiver plot of thrust force angle is corrected to be in the global XYZ frame
% 2021-01-06 YH: sm_boat_run010.m for International Journal of Intelligent Control and Systems 

%% EOMs and boat parameters adapted from Escario et al (2012):
% xdd = ( F*cos(z) - uxl*xd - uxc*xd*abs(xd) ) / ( m + mxx )
% ydd = ( F*sin(z) - uyl*yd - uyc*yd*abs(yd) ) / ( m + myy )
% zdd = ( L/2*F*sin(z) - uzl*L*zd - uzc*zd*abs(zd) ) / ( Iz + Izz )

%% EOMs and boat parameters adapted from Muske et al 2008 and Huang & Ji (2017):
% xdd = ( F*cos(b) + Fd*cos(k-z) + ?m+myy)*yd*zd - d1*sign(xd)*abs(xd)^a1 ) / ( m + mxx )
% ydd = ( F*sin(b) + Fd*sin(k-z) - ?m+mxx)*xd*zd - d2*sign(yd)*abs(yd)^a2 ) / ( m + myy )
% zdd = ( Lt*F*sin(b) -?myy-mxx)*xd*yd - d3*sign(zd)*abs(zd)^a3 ) / ( I + Izz )

% xyz - coordinate for local body-fixed frame
% XYZ - coordinate for global inertial frame fixed to the ground
% x - m, body-fixed longitudinal 'surge' displacement 
% xd - m/s velcoity 
% xdd - m/s2 acceleration
% y - m, body-fixed transverse 'sway' displacement 
% yd - m/s velcoity 
% ydd - m/s2 acceleration
% z - rad, heading 'yaw' angle relative to X-axis of inerital frame
% zd - rad/s ang velcoity 
% zdd - rad/s2 ang acceleration
% b - rad, [-pi/4 pi/4], steering angle of thruster relative to heading (x-axis)
% F - N, [0 20], propulsion force of the thruster 
% k - rad, [-pi pi], disturbance angle relative to inertial frame X-axis
% Fd - N, [0 20], disturbance force acting at mass centre

clear all; clc; 

% 2017 paper to define next waypoint:
% R = 20 ; % m, waypoint radius for trial
% ad = 150 ; % deg
% Xw = R*cosd(ad) 
% Yw = R*sind(ad) 

% wps = [ 40 40; 20 40; 20 200; 50 40; 100 100]; % m, waypoints 'demands' in Earth X- and Y-
% wps = [ -40 -40; 20 40; 20 200; -50 -40; 100 100]; % m, waypoints 'demands' in Earth X- and Y-
aa = 50 ; % m, waypoint dimensional parameter
wps = [0 0; -aa -aa; aa -aa; aa/2 -aa/2; aa/2 aa/2; aa aa; -aa aa; -aa/2 aa/2; -aa/2 0; -aa/5 0]; % m, waypoints 'demands' in Earth X- and Y-
n_o_w = length(wps); % number of waypoints
wp_tol = 1 ; % m, tolerance radius of reaching a waypoint 
wp_sh = 10 ; % m, short distance to waypoint to enble reduced thrust F

% Thrust force
bmax = pi*(0.47) ; % rad, maximum degree of thrust angle beta 'b', full range +/-45 deg 
Fmax = 20 ; % N, maximum thrust force 'F' by all propellers 

% Disturbance force 
% B = 0 ; % N, disurbance force bias
% A = 0 ; % N, disurbance force amplitude
B = 10 ; % N, disurbance force bias
A = 10 ; % N, disurbance force amplitude
fd = 0.1 ; % Hz, disturbance force frequency
% fd = 0 ; % Hz, disturbance force frequency
wd = fd*2*pi ; % rad/s 
kai = pi*2/8 ; % rad, disturbance force angle relative to global frame +X
td = 0 ; % s, delay to start disturbance force - allow boat to build up speed

% Boat parameters
L = 1 ; % m, boat length
Lt = 0.4 ; % m, longitudinal distance between mass centre and thruster force point of action 
W = 0.55 ; % m, boat width 
H = 0.20 ; % m, boat height
Dh = 0.15 ; % m, hull duct diameter for one side of cataraman
D = 0.05 ; % m, mean submerged depth at 7 kg mass
m = 7 ; % kg, boat mass
Iz = m*(W^2+L^2)/12 ; % kgm2, mass moment of inertia about mass centre G in yaw z-axis 
mxx = 0.05*m ; % kg, added mass in forward x-axis of boat - Muske et al 2008
myy = 1000*pi*D^2*L/2 ; % kg, added mass in lateral y-axis of boat  - Muske et al 2008
Izz = (0.1*m*W^2+1000*pi*D^2*L^3)/2/12 ; % kgm2, added mass moment of inertia in yaw z-axis of boat - Muske et al 2008
% d1 = 2.44 ; % forward surge hydrodynamic damping coefficient in SI units - Muske et al 2008
% d2 = 13.0 ; % lateral sway hydrodynamic damping coefficient in SI units - Muske et al 2008
% d3 = 0.0564 ; % yaw hydrodynamic damping coefficient in SI units - Muske et al 2008
% a1 = 1.51 ; % expoential for forward surge hydrodynamic damping - Muske et al 2008
% a2 = 1.75 ; % expoential for lateral sway hydrodynamic damping - Muske et al 2008
% a3 = 1.59 ; % expoential for yaw hydrodynamic damping - Muske et al 2008

% d1 = 14 ; % paper 2017 Huang & Ji
% d2  = 80 ;
% d3 = 5 ;
% a1 = 1 ;
% a2 = 1 ;
% a3 = 1 ;

% d1 = 6.67 ; % paper 2020 Matt McC CFD results - no azipods nor rudders
% d2  = 22.07 ;
% d3 = 2.91 ;
% a1 = 2.19 ;
% a2 = 1.53 ;
% a3 = 1.48 ;

% d1 = 12 ; % YH Oceans 2020
% d2  = 50 ;
% d3 = 6 ;
% a1 = 2 ;
% a2 = 1.5 ;
% a3 = 1.5 ;

% d1 = 8 ; % YH-HY IJICS 2021
% d2  = 50 ;
% d3 = 3 ;
% a1 = 2 ;
% a2 = 1.5 ;
% a3 = 1.5 ;

d1n = [6.67     8   10] ; % YH-HY IJICS 2021
d2n = [22.07    50  60] ;
d3n = [2.91     6   9] ;
a1n = [2.19     3   3] ;
a2n = [1.53     2   2] ;
a3n = [1.48     2   2] ;
for nx = 3:3   
    % for nx = 1:length(d1n)
    d1 = d1n(nx);
    d2 = d2n(nx);
    d3 = d3n(nx);
    a1 = a1n(nx);
    a2 = a2n(nx);
    a3 = a3n(nx);

% Not used in Huang & Ji (2017) paper
% uzl = 10 ; % Nms/rad, linear coefficient of damping to turn
% uzq = 100 ; % Nms2/rad2, quadratic coefficient of damping to turn
% uxl = 1.75 ; % Ns/m, linear coefficient of forward hydrodynamic damping
% uxq = 2.5 ; % Ns2/m2, quadratic coefficient of forward hydrodynamic damping
% uyl = 25 ; % Ns/m, linear coefficient of lateral hydrodynamic damping
% uyq = 250 ; % Ns2/m2, quadratic coefficient of lateral hydrodynamic damping

% Simulink configuration
fixedstep = 0.01 ; % s, simulation time step
fs = 1/fixedstep ; % Hz, sampling frequency 
time = 1200  ; % s, simulation time 

open_system('boatx010') ;
options = simset('Solver','ode4','FixedStep',fixedstep) ;
%     set_param('boatx003/Heading','After','3/4*pi') ;    
[ t , uu , vv ] = sim ( 'boatx010' , time - fixedstep , options ) ;

% Plot parameters
X_lo = floor(min(X.data)-0.1)*1.5 ; % m, scale for plot
X_hi = ceil(max(X.data)+0.1)*1.5 ;
Y_lo = floor(min(Y.data)-0.1)*1.5 ; % m, scale for plot
Y_hi = ceil(max(Y.data)+0.1)*1.5 ;

if (X_hi-X_lo)<(Y_hi-Y_lo)
    X_lo = X_lo ;
    X_hi = X_lo + (Y_hi-Y_lo) ;
elseif (X_hi-X_lo)>(Y_hi-Y_lo)
    Y_lo = Y_lo ;
    Y_hi = Y_lo + (X_hi-X_lo) ;     
end
ut = 300; % s, time scale to plot 
% ut = t(end);

% aa = [t X.data Y.data xd.data] ;
% aai = find((abs(aa(:,2)-Xw)<=wp_tol) & (abs(aa(:,3)-Yw)<=wp_tol));
% aat = aa(aai(1),1);
% vvi = find(aa(:,4)>=max(aa(:,4))*0.99);
% vvmax = max(aa(:,4))
% vvt = aa(vvi(1),1)
% vvd = sqrt(aa(vvi(1),2)^2 + aa(vvi(1),3)^2)
% figure(10);
% plot(t,X.data,'r-',t,Y.data,'b--'); grid; hold on; legend('X','Y');
% line([aat aat],[0 Xw]);
% xlabel('t');ylabel('X and Y ( m )');

figure(1);
subplot(1,2,1)
plot(t,dst.data); xlabel('Time ( s )'); ylabel('dst ( m )'); box on; hold on; % grid on;
xticks([0 100 200 300 400 450]); yticks([0 20 40 60 80 100 120]);
xlim([0 450]); ylim([0 120]);
title('( a )','interpreter','none');
subplot(1,2,2)
plot(Xw.data,Yw.data,'ro',X.data,Y.data);xlabel('X ( m )'); ylabel('Y ( m )'); box on; hold on; % grid on; 
xticks([-60 -40 -20 0 20 40 60]); yticks([-60 -40 -20 0 20 40 60]);
xlim([-60 60]); ylim([-60 60]);
title('( b )','interpreter','none');
sdf('1xrow') ;
end

figure(10); 
axis square
% qin = 20; % samples, quiver plot interval to skip - for visual
qin = 40; % samples, quiver plot interval to skip - for visual
quiver(X.data(1:qin:end),Y.data(1:qin:end),Xd.data(1:qin:end),Yd.data(1:qin:end),...
    'marker','.','linewidth',1.25,'MaxHeadSize',0.6,'AutoScaleFactor',0.4,'color','k'); hold on;
quiver(X.data(1:qin:end),Y.data(1:qin:end),L*cos(z.data(1:qin:end)),L*sin(z.data(1:qin:end)),...
    'marker','.','linewidth',1.25,'MaxHeadSize',0.6,'AutoScaleFactor',0.4,'color','b'); hold on;
quiver(X.data(1:qin:end),Y.data(1:qin:end),F.data(1:qin:end).*cos(b.data(1:qin:end)+z.data(1:qin:end)),...
    F.data(1:qin:end).*sin(b.data(1:qin:end)+z.data(1:qin:end)),...
    'marker','.','linewidth',1.25,'MaxHeadSize',0.6,'AutoScaleFactor',0.4,'color','r'); 
% Thrust angle b is measured in the local (boat body) frame, so use (b+z)
% to indicate thrust angle in the global (inertial) frame
% xlim([-2*aa 2*aa]); ylim([-2*aa 2*aa]); % full view
xlim([40 60]); ylim([-60 -40]);xticks([40 50 60]); yticks([-60 -50 -40]); % zoom in
xlabel('X ( m )'); ylabel('Y ( m )');
hold on; grid on; 
% quiver(X.data,Y.data,gradient(X.data),gradient(Y.data),'b-'); grid on;

scatter(wps(:,1),wps(:,2),400,'^','b','filled');box on; grid on;
a = [1:n_o_w]'; d = num2str(a); c = cellstr(d);
dx = 5; dy = 5;
text(wps(:,1)+dx,wps(:,2)+dy,c,'color','b');
legend('Boat velocity (m/s)','Boat heading z','Thrust force F and direction b','Waypoints')

% scatter(obs(:,1),obs(:,2),'o','filled','r');
% f = [1:n_o_o]'; g = num2str(f); h = cellstr(g);
% dx = 5; dy = 5;
% text(obs(:,1)+dx,obs(:,2)+dy,h,'color','r');

% for i = 1:length(obs); % draw circle around each obstacle - why?
%     rectangle('Position',[obs(i,1)-RO, obs(i,2)-RO, RO*2, RO*2],'Curvature',[1,1]);
% end
sdf('auto boat path') ; % 3rd party function save figure existing export style 'f_m_a x3'

figure(20);
subplot(2,2,1)
% plot(t,z.data*180/pi,'k-',t,hd.data*180/pi,'k-.',t,dhd.data*180/pi,'k--',t,b.data*180/pi,'k:');hold on; box on; grid on;
% legend('Current heading z (deg)','Demand heading hd (deg)','Heading difference dhd = hd - z (deg)',...
%     ['Thrust angle b, ' 'disturbance angle k = ' num2str(k.data(5)*180/pi) ' deg']);

% plot(t,z.data*180/pi,'k-',t,hd.data*180/pi,'k-.',t,dhd.data*180/pi,'k--');hold on; box on; grid on;
% legend('z (deg)','hd (deg)','dhd = hd - z (deg)',...
%     ['Thrust b, ' 'disturbance k = ' num2str(k.data(1)*180/pi) ' deg']);

plot(t,z.data*180/pi,'k-',t,b.data*180/pi,'k:');hold on; box on; grid on;  
% legend('Heading z (yaw)', ['Thrust b, ' 'disturbance k = ' num2str(k.data(1)*180/pi) ' deg']);
legend('Heading z (yaw)', 'Thrust b');

xlim([0 ut]);ylabel('Angle ( deg )');xlabel('Time ( s )');grid on;box on;
title('( a )','interpreter','none');

subplot(2,2,2)
plot(X.data,Y.data,'k-'); xlabel('X (m)'); ylabel('Y (m)'); grid on; box on; hold on;
plot(0,0,'bo',wps(:,1),wps(:,2),'r+'); hold on;
r = wp_tol ; x = wps(:,1) ; y = wps(:,2) ; theta = linspace(0,2*pi) ; 
% plot(r*cos(theta)+x , r*sin(theta)+y , 'r--') ; hold on;
axis([X_lo X_hi Y_lo Y_hi]);
title('( b )','interpreter','none');
% text(Xw,Yw+wp_tol,['( ' num2str(Xw) ' , ' num2str(Yw) ' )']);
% plot(X.data,Y.data,'b-');axis([X_lo X_hi Y_lo Y_hi]);
% ylabel('Y ( m )','interpreter','none');xlabel('X ( m )','interpreter','none');grid on;box on;
% title('Earth inertial frame');

subplot(2,2,3)
plot(t,xd.data,'k-',t,yd.data,'k--',t,zd.data,'k:');
xlim([0 ut]);axis([0 ut -3 3]);xlabel('Time ( s )');ylabel('Speed');grid on;box on;
legend('Speed x: Surge (m/s)','Speed y: Sway (m/s)','Speed z: Yaw (rad/s)');
title('( c )','interpreter','none');

subplot(2,2,4)
% plot(t,xd.data,'b-',t,yd.data,'r-',t,zd.data,'b:',t,F.data,'k-',t,Fd.data,'k:');
plot(t,F.data,'k-',t,Fd.data,'k--');
xlim([0 ut]);axis([0 ut 0 50]);xlabel('Time ( s )');ylabel('Force ( N )');grid on;box on;
legend('Force F','Disturbance force Fd','Location','best');
title('( d )','interpreter','none');

sdf('auto boat states 2x2');
% sdf('auto boat path') ; % 3rd party function save figure existing export style 'f_m_a x3'




