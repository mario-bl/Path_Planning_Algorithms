function dtheta = angleDiff(theta1, theta2)
    dtheta = theta1 - theta2;
    % Normalizamos a [-pi, pi]
    dtheta = mod(dtheta + pi, 2*pi) - pi;
end