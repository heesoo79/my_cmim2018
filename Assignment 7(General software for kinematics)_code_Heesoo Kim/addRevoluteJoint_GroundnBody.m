function [ C ] = addRevoluteJoint_GroundnBody( x, y, phi, body_length, joint_position )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
C(1) = x - body_length/2*cos(phi) - joint_position(1);
C(2) = y - body_length/2*sin(phi) - joint_position(2);
end

