function q = euler_to_quat(eul)
%EULER_TO_QUAT Convert [roll pitch yaw] to PX4 quaternion [w x y z].
roll = eul(1); pitch = eul(2); yaw = eul(3);

cr = cos(roll * 0.5); sr = sin(roll * 0.5);
cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);

q = [
    cr * cp * cy + sr * sp * sy;
    sr * cp * cy - cr * sp * sy;
    cr * sp * cy + sr * cp * sy;
    cr * cp * sy - sr * sp * cy
];
end
