node = ros2node('aruco_pose_listener');

sub = ros2subscriber(node, '/aruco_pose', 'geometry_msgs/Pose', @callback);

function callback(msg)
    % Wyodrębnienie pozycji
    x = msg.position.x;
    y = msg.position.y;
    z = msg.position.z;
    
    qx = msg.orientation.x;
    qy = msg.orientation.y;
    qz = msg.orientation.z;
    qw = msg.orientation.w;
    
    angles = quat2eul([qw qx qy qz]);
    roll = rad2deg(angles(1));
    pitch = rad2deg(angles(2));
    yaw = rad2deg(angles(3));
    
    fprintf('x = %.2f m, y = %.2f m, z = %.2f m\n', x, y, z);
    fprintf('roll = %.2f°, pitch = %.2f°, yaw = %.2f°\n\n', roll, pitch, yaw);
end

