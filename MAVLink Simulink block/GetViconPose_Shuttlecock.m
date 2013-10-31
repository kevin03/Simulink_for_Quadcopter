function [ViconPose] = GetViconPose(u)
global MyClient;
global QuadRotationMatix;

while (MyClient.GetFrame().Result.Value) ~= (Result.Success)
	fprintf( '.' );
end 


% *********************************
% x240
% *********************************
% SubjectName = 'x240'; SegmentName = 'x240';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('x240', 'x240');
ViconPose(1) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(2) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(3) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('x240', 'x240');
ViconPose(4) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(5) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(6) = QuadEulerXYZ.Rotation(3); % Yaw


% *********************************
% x240_2
% *********************************
% SubjectName = 'x240_2'; SegmentName = 'x240_2';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('x240_2', 'x240_2');
ViconPose(7) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(8) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(9) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('x240_2', 'x240_2');
ViconPose(10) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(11) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(12) = QuadEulerXYZ.Rotation(3); % Yaw


end 




