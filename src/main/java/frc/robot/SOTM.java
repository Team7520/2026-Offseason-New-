package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

// The code in here is for shoot on the move
// One way we could do it is by simulating how long the shot would take if we were stationary, move the robot 
// by its velocity vector * time, and pretend we are taking the shot from there.

class SOTM {

    public Rotation2d computeAzimuthStatic(Pose2d robotPose, Pose2d goalPose) {
        Transform2d delta = goalPose.minus(robotPose);

        Rotation2d absoluteAzimuth = delta.getTranslation().getAngle();
        Rotation2d robotHeading = robotPose.getRotation();

        Rotation2d relativeAzimuth = absoluteAzimuth.minus(robotHeading);
        return relativeAzimuth;
    }

}