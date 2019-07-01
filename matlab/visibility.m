%==========================================================================
% Computation of visibility conditions.
% Visibility only when elevation is above threshold and at night.
% Author: Marilena Di Carlo, 2018. marilena.di-carlo@strath.ac.uk
% Reference: Vallado - Fundamentals of Astrdynamics and Applications
% =========================================================================

% Input:
%       orbit     -> structure with the keplerian elements of the spacecraft orbit
%                 orbit.a: semimajor axis [km]; orbit.e: eccentricity;
%                 orbit.i: inclination [rad]; orbit.omega: argument of perigee [rad];
%                 orbit.RAAN: right ascension of the ascending node [rad];
%                 orbit.theta0: initial true anomaly [red]
%       time      -> array of times when the visibility has to be computed [days]
%       min_El    -> minimum elevation for visibility [rad]
%       GS        -> structure containing latitude and longitude of the ground stations
%       constants -> structure containing constants for the problem
%
% Output:
%       vis_time
%       El
%       diff_start_time_vis
%       RAAN


function [visibility_flag, El, RAAN] = visibility(orbit, time, ...
    min_El, GS, constants)


%% Initialise spacecraft orbit variables

% Mean motion
n = sqrt(constants.mu / orbit.a^3);

% Orbital period
T = 2 * pi / n;

% Velocity on circular orbit
omega = 2*pi/T;

% Semilatus rectum
p = orbit.a * (1 - orbit.e^2);

% Variation of RAAN due to J2
RAAN_dot = -3/2 * n * constants.J2 * (constants.R_Earth_eq / p)^2 * cos(orbit.i);


stations = fields(GS);

for i = 1 : numel(time)
    
    disp([num2str(i), ' out of ', num2str(numel(time))])
    
    %% Compute cartesian position spacecraft
    
    % Update theta
    theta = orbit.theta0 + omega * time(i) * constants.sec_day;
    theta = mod(theta, 2*pi);
    
    % Update RAAN
    RAAN_tmp = orbit.RAAN0 + RAAN_dot * time(i) * constants.sec_day;
    RAAN(i) = mod(RAAN_tmp, 2*pi);
    
    % Keplerian elements
    kep = [orbit.a, orbit.e, orbit.i, RAAN(i), orbit.omega, theta];
    
    % Cartesian elements
    cart = kep2cart(kep, constants.mu);
    r_SC = cart(1:3);
    
    %% Compute position GS in ECI
    
    % JD
    JD = mjd20002jd(orbit.t0 + time(i));
    
    % GMST
    GreenMST  =  GMST(JD);
    
  
    for index_GS = 1 : numel(stations)
        
       % Current station
       current_GS = stations{index_GS};
        
        % Local sidereal time
        LST = mod(GreenMST  + GS.(current_GS).Long, 2*pi);
        
        % Position of the GS in ECI
        r_GS  =  GS_position_ECI( GS.(current_GS), LST, constants );
        
        %% Compute vector GS-SC in ECI and SEZ
        
        % Vector GS to SC in ECI
        r_GS_SC = r_SC - r_GS;
        
        % Transformation from ECI to SEZ
        R = [sin(GS.(current_GS).GeodLat)*cos(LST), sin(GS.(current_GS).GeodLat)*sin(LST), ...
            -cos(GS.(current_GS).GeodLat); ...
            -sin(LST), cos(LST), 0; ...
            cos(GS.(current_GS).GeodLat)*cos(LST), cos(GS.(current_GS).GeodLat)*sin(LST), ...
            sin(GS.(current_GS).GeodLat)];
        
        % Vector GS to SC in SEZ
        r_GS_SC = R * r_GS_SC';
        
        % Elevation of the SC wrt GS
        El(index_GS, i) = asin(r_GS_SC(3) / norm(r_GS_SC));
        
        %% Date and time
        
        % Current date in date
        current_date = jd2date(JD);
        
        % Elapsed days from beginnning of year
        elaps_days = JD - date2jd([current_date(1) 1 1 0 0 0]);
        
        % Interpolation to get sunrise and sunset time
        elaps_sec_sunrise = interp1(GS.(current_GS).elapsed_days, ...
            GS.(current_GS).sunrise_elapsed_sec, elaps_days);
        
        elaps_sec_sunset = interp1(GS.(current_GS).elapsed_days, ...
            GS.(current_GS).sunset_elapsed_sec, elaps_days);
        
        % Transform elapsed seconds in current sunrise time and sunset time
        beg_day = [current_date(1) current_date(2) current_date(3) 0 0 0];
        JD_beg_day = date2jd(beg_day);
        
        sunrise_time_JD = JD_beg_day + elaps_sec_sunrise / constants.sec_day;
        sunrise_time = jd2date(sunrise_time_JD);
        
        sunset_time_JD = JD_beg_day + elaps_sec_sunset / constants.sec_day;
        sunset_time = jd2date(sunset_time_JD);
        
        % Visibility only if elevation is greater than limit and if time is
        % past sunset
        if El(index_GS, i) > min_El 
            
            % If is night...
            if ( (current_date(4) > sunset_time(4) ||  current_date(4) < sunrise_time(4) ) || ...
                    (current_date(4) == sunset_time(4) && current_date(5) >= sunset_time(5)) || ...
                    (current_date(4) == sunrise_time(4) &&  current_date(5) <= sunrise_time(5)) )
                
                visibility_flag(index_GS, i) = 1;

            end
            
            % If elevation is lower than limit
        else
            visibility_flag(index_GS, i) = 0;
        end
        
    end
    
end
