function [ C ] = addRevoluteJoint_BodynBody( x1, y1, phi1, x2, y2, phi2, body1_length, body2_length )
% UNTITLED4 Summary of this function goes here
% Detailed explanation goes here
C(1) = x1 + body1_length/2*cos(phi1) - x2 + body2_length/2*cos(phi2);
C(2) = y1 + body1_length/2*sin(phi1) - y2 + body2_length/2*sin(phi2);
end

