package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

// The code in here is for shoot on the move
// One way we could do it is by simulating how long the shot would take if we were stationary, move the robot 
// by its velocity vector * time, and pretend we are taking the shot from there.

class SOTM {

    public Rotation2d computeAzimuthStatic(Pose2d robotPose, Pose2d goalPose) {
        Translation2d delta = goalPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d absoluteAzimuth = delta.getAngle();
        Rotation2d robotHeading = robotPose.getRotation();
        return absoluteAzimuth.minus(robotHeading);
    }

}