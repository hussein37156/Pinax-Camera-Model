% ------------------------------------------------------------------------
% This empirical equation for finding the refraction index of sea water 
% given light wavelength and readings of salinity and temperature was 
% published by the authors X. Quan and E. S. Fry in 1995.
% 
% “Empirical equation for the index of refraction of seawater,” 
% Applied Optics, vol. 34, no. 18, p. 3477, 1995. doi:10.1364/ao.34.003477
%
% Author of this code: Adrienne Winter, African Robotics Unit, University of Cape Town, 2023.
% ------------------------------------------------------------------------

S = 0.0000324; % salinity reading in parts-per-million (ppm or ‰)
T = 20.9; % temperature reading in degrees celcius
lambda = 587; % light wavelength in nanometers

n = 1.31405 + (1.779e-4 + (-1.05e-6)*T + (1.6e-8)*T^2)*S - (2.02e-6)*T^2 + ((15.868 + 0.01155*S - 0.00423*T)/lambda) - 4382/lambda^2 + (1.1455e6)/lambda^3 