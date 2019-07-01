% =========================================================================
% Ground station position vector in ECI reference frame.
% Author: Marilena Di Carlo, 2018. marilena.di-carlo@strath.ac.uk
% Reference: Vallado - Fundamentals of Astrdynamics and Applications
%           (pag.408 and pag.137)
% ========================================================================

% Input:
%       GS        -> structure with latitude and longitude of ground station
%       LST       -> Local Sideral Time
%       constants -> structure with constants for the problem

% Output:
%           r --> ground station position vector (ECI coordinate system)


function r  =  GS_position_ECI( GS, LST, constants )

% Earth Eccentricity
ecc_EARTH  =  sqrt( 1  -  (constants.R_Earth_polar / constants.R_Earth_eq )^2  );

% Vallado, pag.144 :
C_EARTH  =  constants.R_Earth_eq  / sqrt( 1  -  (ecc_EARTH * sin(GS.GeodLat))^2);
S_EARTH  =  C_EARTH * ( 1  -  ecc_EARTH^2 );

% Ground station coordinates:
x  =  (C_EARTH + GS.Alt) * cos(GS.GeodLat) * cos(LST);
y  =  (C_EARTH + GS.Alt) * cos(GS.GeodLat) * sin(LST);
z  =  (S_EARTH + GS.Alt) * sin(GS.GeodLat);

r = [x y z];


end
