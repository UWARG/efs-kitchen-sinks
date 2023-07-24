function align = fcn(diff, lat, long)

    % antenna is the cartesian coordinates of the tracking antenna
    % where the origin is the center of the earth
    % z is the north pole
    % x and y are on the equator

    % diff is the cartesian coordinates of the drone relative to the
    % tracking antenna

    % want to find the difference between the drone and the antenna
    % in the antenna's local cartesian coordinates
    % where z is up, x is east, and y is north

    % first create the rotation matrix using the latitude and longitude
    % use rotx roty and rotz to create the rotation matrix
    rot = rotz(-long)*roty(-lat)

    % third, rotate the drone's coordinates to the antenna's
    % coordinates
    align = rot * diff;
    
