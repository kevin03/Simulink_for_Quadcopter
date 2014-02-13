function [ViconPose] = GetViconPose(u)
global MyClient;
global QuadRotationMatix;

while (MyClient.GetFrame().Result.Value) ~= (Result.Success)
	fprintf( '.' );
end 

% *********************************
% Joystick Controlled Quad
% *********************************
% SubjectName = 'k1'; SegmentName = 'k1';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('test1', 'test1');
ViconPose(1) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(2) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(3) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('test1', 'test1');
ViconPose(4) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(5) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(6) = QuadEulerXYZ.Rotation(3); % Yaw


% *********************************
% Formation Quad 1
% *********************************
% SubjectName = 'k1'; SegmentName = 'k1';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('AS_330_1', 'AS_330_1');
ViconPose(7) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(8) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(9) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('AS_330_1', 'AS_330_1');
ViconPose(10) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(11) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(12) = QuadEulerXYZ.Rotation(3); % Yaw


% *********************************
% Formation Quad 2
% *********************************
% SubjectName = 'k1'; SegmentName = 'k1';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('AS_330_2', 'AS_330_2');
ViconPose(13) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(14) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(15) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('AS_330_2', 'AS_330_2');
ViconPose(16) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(17) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(18) = QuadEulerXYZ.Rotation(3); % Yaw

% *********************************
% Formation Quad 3
% *********************************
% SubjectName = 'k1'; SegmentName = 'k1';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('AS_330_3', 'AS_330_3');
ViconPose(19) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(20) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(21) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('AS_330_3', 'AS_330_3');
ViconPose(22) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(23) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(24) = QuadEulerXYZ.Rotation(3); % Yaw

end 



