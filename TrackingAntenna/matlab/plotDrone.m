% plot out.dronegps (which is a timeseries of LLA) on a map using geoplot

% geoplot only takes two points at a time, so I need to loop through the
% timeseries and plot each point

% out.dronegps(1).Data is 1x3xN, where N is the number of points
% we need to get a Nx1 vector of latitudes and longitudes

lats = out.dronegps.Data(1,1,:);
lons = out.dronegps.Data(1,2,:);

% make them vectors
lats = lats(:);
lons = lons(:);

% the tracking antenna has LLA at out.antennaOrigin
% the antenna's yaw is a vector of angles out.antennaYaw
% want to plot where the antenna is pointing at each time step
% a yaw of 0 means the antenna is pointing east, with positive yaw being CCW

heading = out.targetHeading.Data;
antenna_origin = out.antennaOrigin.Data;
antenna_lat = antenna_origin(1,1);
antenna_lon = antenna_origin(1,2);
antenna_alt = antenna_origin(1,3);

geoplot(lats, lons, 'r', antenna_lat, antenna_lon, 'b*')

% slice = 2000;
% plot the antenna's yaw at point 10000
% [lat, lon] = yaw2latlon(antenna_lat, antenna_lon, heading(slice));
% geoplot([antenna_lat lat], [antenna_lon lon], 'b', [antenna_lat], [antenna_lon], 'b*', lats(1:slice), lons(1:slice), 'r')

% out.droneCartesian is a timeseries of Nx3 vectors of the drone's position
% plot it in 3D space

% plot3(out.droneCartesian.Data(:,1), out.droneCartesian.Data(:,2), out.droneCartesian.Data(:,3))

% out.diffCartesian is a timeseries of Nx3 vectors of the difference between
% the drone's position and the antenna's position
% where 


function [lat, lon] = yaw2latlon(antenna_lat, antenna_lon, heading)
    lineLength = 1; % km
    lineLength = km2deg(lineLength); % degrees

    % % convert the heading to a unit vector with magnitude 10, where x is
    % % north and y is east
    % heading_vector = [cosd(heading) sind(heading)] * 10;

    % % convert heading from unit vector in cartesian coordinates to arclen and azimuth
    % [arclen, azimuth] = cart2sph(heading_vector(1), heading_vector(2), 0);

    % convert vector to lat/lon
    [lat, lon] = reckon(antenna_lat, antenna_lon, lineLength, heading);
end

