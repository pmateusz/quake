%==========================================================================
% Computation of the Greenwich Mean Sidereal Time (GMST).
% Author: Marilena Di Carlo, 2018. marilena.di-carlo@strath.ac.uk
% Reference: Vallado - Fundamentals of Astrdynamics and Applications
%           (pag. 192)
% =========================================================================

% Input:
%           JD --> Julian Date

% Output:
%           Greenwich_Mean_Sidereal_Time --> Greenwich Mean Sidereal Time
%                                            at time identified by JD [rad]


function Greenwich_MST  =  GMST(JD)

%Computation of Universal Time
tUT  =  (JD - 2451545) / 36525;

%Computation of Greenwich Mean Sidereal Time
GMST_1  =  67310.54841  +  (  ( 876600 * 3600 + 8640184.812866 ) *tUT )  +  ...
    ( 0.093104 * (tUT^2) )  -  ( ( 6.2 * 10^(-6) ) * (tUT^3) );
GMST_2  =  fix( GMST_1 / 86400 );
GMST_3  =  GMST_1  -  86400 * GMST_2;
GMST_4  =  GMST_3 / 240;

GMST_4 = mod(GMST_4, 360);

Greenwich_MST = GMST_4 * pi/180;
