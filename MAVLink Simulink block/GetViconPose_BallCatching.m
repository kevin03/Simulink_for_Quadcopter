function [ViconPose] = GetViconPose(u)
global MyClient;
global QuadRotationMatix;

while (MyClient.GetFrame().Result.Value) ~= (Result.Success)
	fprintf( '.' );
end 



% *********************************
% x330_1
% *********************************
% SubjectName = 'x330'; SegmentName = 'x330';

% X, Y, Z Global Coordinates of the Quad
Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('MAV2', 'MAV2');
ViconPose(1) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(2) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(3) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

% Euler Angles
QuadEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ('MAV2', 'MAV2');
ViconPose(4) = QuadEulerXYZ.Rotation(1); % Roll
ViconPose(5) = QuadEulerXYZ.Rotation(2); % Pitch
ViconPose(6) = QuadEulerXYZ.Rotation(3); % Yaw

Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation('basket', 'basket');
ViconPose(7) = Output_GetSegmentGlobalTranslation.Translation(1);  % X-coordinate
ViconPose(8) = Output_GetSegmentGlobalTranslation.Translation(2);  % Y-coordinate
ViconPose(9) = Output_GetSegmentGlobalTranslation.Translation(3);  % Z-coordinate

%Unlabeled Marker
Output = MyClient.GetUnlabeledMarkerGlobalTranslation(1);
ViconPose(10) = Output.Translation(1);  % X-coordinate
ViconPose(11) = Output.Translation(2);  % Y-coordinate
ViconPose(12) = Output.Translation(3);  % Z-coordinate

Output = MyClient.GetUnlabeledMarkerGlobalTranslation(2);
ViconPose(13) = Output.Translation(1);  % X-coordinate
ViconPose(14) = Output.Translation(2);  % Y-coordinate
ViconPose(15) = Output.Translation(3);  % Z-coordinate

% X, Y, Z Global Coordinates
Output = MyClient.GetUnlabeledMarkerGlobalTranslation(3);
ViconPose(16) = Output.Translation(1);  % X-coordinate
ViconPose(17) = Output.Translation(2);  % Y-coordinate
ViconPose(18) = Output.Translation(3);  % Z-coordinate

end 



