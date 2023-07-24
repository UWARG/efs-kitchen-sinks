function [angle, counter] = fcn(newAngle, lastAngle, lastCounter)
    % prevent angle from wrapping around 360 or 0 degrees
    % by counting the number of times the angle wraps, and
    % adding that to the angle

    if newAngle > (lastAngle + 180)
        counter = lastCounter + 1;
    elseif newAngle < (lastAngle - 180)
        counter = lastCounter - 1;
    else
        counter = lastCounter;
    end

    % it should be counter but that somehow causes a spike in the data
    % lastCounter seems to work better
    % idk why but it works
    angle = newAngle + lastCounter * 360;
end
