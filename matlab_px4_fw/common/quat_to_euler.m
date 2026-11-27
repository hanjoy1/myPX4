function eul = quat_to_euler(q)
%QUAT_TO_EULER Convert PX4 quaternion [w x y z] to [roll pitch yaw].
qw = q(1); qx = q(2); qy = q(3); qz = q(4);

sinr_cosp = 2.0 * (qw * qx + qy * qz);
cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
roll = atan2(sinr_cosp, cosr_cosp);

sinp = 2.0 * (qw * qy - qz * qx);
if abs(sinp) >= 1.0
    pitch = sign(sinp) * pi / 2.0;
else
    pitch = asin(sinp);
end

siny_cosp = 2.0 * (qw * qz + qx * qy);
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
yaw = atan2(siny_cosp, cosy_cosp);

eul = [roll; pitch; yaw];
end
