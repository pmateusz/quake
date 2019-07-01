clear
close all
clc

% Addpath to smart-astro - Files used from smart-astro:
% date2mjd2000, kep2cart, mjd20002jd, jd2date, date2jd
addpath(genpath('C:\Users\Marilena\smart-astro\MATLAB\spaceart_toolbox'))


%% Load sunrise and sunset times - substitute with command to read from file?
sunrise_sunset

%% Constants

% Earth equatorial radius [km]
constants.R_Earth_eq = 6378.136;
% Earth polar radius [km]
constants.R_Earth_polar = 6356.751;
% J2
constants.J2 = 1.0826e-3;
% Earth's gravitational constant [km^2/s^2]
constants.mu = 398600;
% Number of seconds in a day
constants.sec_day = 86400;


%% Orbit and time

% Orbital parameters
orbit.a     = constants.R_Earth_eq + 674;
orbit.e     = 0;
orbit.i     =  90 * pi/180;
orbit.omega = 0;
orbit.theta0 = 0;
orbit.RAAN0  = pi/2;

% Initial date
date.year  = 2030;
date.month = 12;
date.day   = 21;
date.hour  = 0;
date.min   = 0;
date.sec   = 0;

% Time of flight for the simulation [days]
ToF = 1;

% Number of time intervals
t_step = 86400;

% Vector of time
time = linspace(0, ToF, t_step);


%% Ground stations

% London
GS.London.GeodLat = 51.5074 * pi/180;
GS.London.Long    = 0.1278 * pi/180;
GS.London.Alt     = 0;
GS.London.elapsed_days        = London.elapsed_days;
GS.London.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.London.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Glasgow
GS.Glasgow.GeodLat = 55.8642 * pi/180;
GS.Glasgow.Long    = 4.2518 * pi/180;
GS.Glasgow.Alt     = 0;
GS.Glasgow.elapsed_days        = London.elapsed_days;
GS.Glasgow.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Glasgow.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Thurso
GS.Thurso.GeodLat = 58.5936 * pi/180;
GS.Thurso.Long    = 3.5221 * pi/180;
GS.Thurso.Alt     = 0;
GS.Thurso.elapsed_days        = London.elapsed_days;
GS.Thurso.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Thurso.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Manchester
GS.Manchester.GeodLat = 53.4808 * pi/180;
GS.Manchester.Long    = 2.2426 * pi/180;
GS.Manchester.Alt     = 0;
GS.Manchester.elapsed_days        = London.elapsed_days;
GS.Manchester.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Manchester.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Birmingham
GS.Birmingham.GeodLat = 52.4862 * pi/180;
GS.Birmingham.Long    = 1.8904 * pi/180;
GS.Birmingham.Alt     = 0;
GS.Birmingham.elapsed_days        = London.elapsed_days;
GS.Birmingham.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Birmingham.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Bristol
GS.Bristol.GeodLat = 51.4545 * pi/180;
GS.Bristol.Long    = 2.5879 * pi/180;
GS.Bristol.Alt     = 0;
GS.Bristol.elapsed_days        = London.elapsed_days;
GS.Bristol.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Bristol.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Ipswich
GS.Ipswich.GeodLat = 52.0567 * pi/180;
GS.Ipswich.Long    = mod(-1.1482, 360) * pi/180; 
GS.Ipswich.Alt     = 0;
GS.Ipswich.elapsed_days        = London.elapsed_days;
GS.Ipswich.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Ipswich.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% Cambridge
GS.Cambridge.GeodLat = 52.2053 * pi/180;
GS.Cambridge.Long    = mod(-0.1218, 360) * pi/180;
GS.Cambridge.Alt     = 0;
GS.Cambridge.elapsed_days        = London.elapsed_days;
GS.Cambridge.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.Cambridge.sunset_elapsed_sec  = London.sunset_elapsed_sec;

% York
GS.York.GeodLat = 53.9600 * pi/180;
GS.York.Long    = 1.0873 * pi/180;
GS.York.Alt     = 0;
GS.York.elapsed_days        = London.elapsed_days;
GS.York.sunrise_elapsed_sec = London.sunrise_elapsed_sec;
GS.York.sunset_elapsed_sec  = London.sunset_elapsed_sec;

%% Other input

% Minimum elevation
min_El = 10 * pi/180;

%% Compute visibility

% Conversion of date in MJD2000
t0 = date2mjd2000([date.year date.month date.day date.hour date.min date.sec]);
orbit.t0 = t0;

% Computation of the visibility
disp('Computation of the visibility conditions...')
[visibility_flag, El, RAAN] = visibility(orbit, time, min_El, ...
    GS, constants);



%% Results for London

i_plot = 1;

% Index of time when GS is visible from SC
index_vis_times = find((visibility_flag(i_plot,:)==1));

% Times when GS is visible from SC
times_visibility = time(index_vis_times);

% Identify the begin and end of each visibility period [days]
time_step = ToF/ t_step;
count = 1;
start_time(count) = times_visibility(1);
for ij = 1 : numel(times_visibility)-1
    if times_visibility(ij+1) - times_visibility(ij) > 10*time_step 
        
    ij

        end_time(count) = times_visibility(ij);
        start_time(count+1) = times_visibility(ij+1);
        count = count + 1;
    end
end
end_time(count) = times_visibility(end);

% Duration of the visibiility windows [min]
duration = (end_time - start_time) * constants.sec_day / 60;


% Plot for London
figure
subplot(2,1,1)
plot(time, visibility_flag(i_plot,:),'LineWidth',2)
grid on
xlabel('Time [days]')
ylabel('Visibility conditions')
subplot(2,1,2)
plot(time, El(i_plot,:)*180/pi,'LineWidth',2)
hold on
plot([time(1) time(end)],[min_El*180/pi min_El*180/pi],'LineWidth',2,...
    'Color','r')
grid on
xlabel('Time [days]')
ylabel('Elevation [deg]')






