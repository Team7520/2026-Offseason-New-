package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class GoToAzimuth extends Command{
    TurretSubsystem turret;
    Pose2d robot, goal;
    
    public  GoToAzimuth(Pose2d robot, Pose2d goal, TurretSubsystem turret) {
        this.turret = turret;
        this.robot = robot;
        this.goal = goal;
    }

    public Rotation2d computeAzimuthStatic(Pose2d robotPose, Pose2d goalPose) {
        Translation2d delta = goalPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d absoluteAzimuth = delta.getAngle();
        Rotation2d robotHeading = robotPose.getRotation();
        return absoluteAzimuth.minus(robotHeading);
    }

    @Override
    public void execute() {
        Rotation2d azimuth = computeAzimuthStatic(robot, goal);
        turret.setAzimuth(azimuth.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopAll();
    }
}
