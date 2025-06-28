
angleSpan = 160;

ServoMiddleAngle = 90;

ServoAngleSpan_per_SteeringWheelAngle = 160/160;

steeringDegrees = 58;

rowValue = ServoMiddleAngle + (steeringDegrees * ServoAngleSpan_per_SteeringWheelAngle)
rowValue = ServoMiddleAngle - (steeringDegrees * ServoAngleSpan_per_SteeringWheelAngle)

% 120 middle    30 deg
% 32 right              58 deg


cmd = 2216 * 0.034321 / 2.0

plot([1 2], [3 4], "-.s")



