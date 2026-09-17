package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class GoToAzimuth extends Command{
    TurretSubsystem turret;
    Supplier<Pose2d> robotPoseSupplier;
    Pose2d goal;
    
    public  GoToAzimuth(Supplier<Pose2d> robotPoseSupplier, Pose2d goal, TurretSubsystem turret) {
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.goal = goal;
    }

    @Override
    public void execute() {
        Rotation2d azimuth = turret.calculateTurretAzimuth(robotPoseSupplier.get(), goal);
        turret.setTurretAzimuth(azimuth);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopAll();
    }
}
