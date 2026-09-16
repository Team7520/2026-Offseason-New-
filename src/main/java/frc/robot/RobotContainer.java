// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.UniverseConstants;
import frc.robot.commands.ExtendAndRunIntake;
import frc.robot.commands.GoToAzimuth;
import frc.robot.commands.IntakeAgitation;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ReverseWheels;
import frc.robot.commands.ShootAndIndex;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem; 
import frc.robot.subsystems.DyerotorSubsystem; 
import frc.robot.subsystems.IntakeSubsystem; 

public class RobotContainer {
    // Subsystems
    private final TurretSubsystem turret;
    private final DyerotorSubsystem dyerotor;
    private final IntakeSubsystem intake;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);


    private double MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        turret = new TurretSubsystem(drivetrain);
        dyerotor = new DyerotorSubsystem();
        intake = new IntakeSubsystem();

        // Configure the button bindings
        configureBindings();
    }

    // public void setLocation(List<EstimatedRobotPose> visionEsts) {
    //     for (var est : visionEsts) {
    //         Pose2d pose = est.estimatedPose.toPose2d();
    //         drivetrain.addVisionMeasurement(pose, est.timestampSeconds);
    //     }
    //     System.out.println(drivetrain.getPose());
    // }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // default commands

        // turret.setDefaultCommand (
        //     new GoToAzimuth(drivetrain::getPose, UniverseConstants.redGoalPose.toPose2d(), turret)
        // );

        turret.setDefaultCommand (
            turret.autoAim()
        );

        // driver commands

        // driver.rightTrigger().whileTrue(
        //     new ParallelCommandGroup(new ShootAndIndex(dyerotor, turret),
        //     turret.shootCommand())
        // );

        driver.rightTrigger().whileTrue(
            new ShootAndIndex(dyerotor, turret)
        );

        driver.leftTrigger().whileTrue(
            new ExtendAndRunIntake(intake, 0.9)
        );

        driver.rightBumper().whileTrue(
            intake.spinRoller(-0.9)
        ).onFalse(
            new InstantCommand(() -> intake.stopAll()) 
        );

        driver.leftBumper().whileTrue(
            new RetractIntake(intake, 0.3)
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );

        driver.x().whileTrue(
            new InstantCommand(() -> turret.turn(0.2)))
        .onFalse(new InstantCommand(() -> turret.stopAll())
        );

        driver.y().whileTrue(
            new InstantCommand(() -> turret.turn(-0.2)))
        .onFalse(new InstantCommand(() -> turret.stopAll())
        );        

        driver.a().whileTrue(new GoToAzimuth(drivetrain::getPose, UniverseConstants.redGoalPose.toPose2d(), turret));

        driver.povUp().onTrue(
            intake.blockerToggle()
        ).onFalse(new InstantCommand(turret::stopAll));

        driver.povDown().whileTrue(
            intake.spinRoller(0.9))
        .onFalse(
            new InstantCommand(() -> intake.stopAll())
        );

        driver.povLeft().whileTrue(
            turret.turnHood(0.1)
        ).onFalse(
            new InstantCommand(() -> turret.stopAll())
        );

        driver.povRight().whileTrue(
            turret.turnHood(-0.1)
        ).onFalse(
            new InstantCommand(() -> turret.stopAll())
        );


// Final x command: x-cross wheels
        // drive.x().whileTrue(
        // drivetrain.applyRequest(() -> brake)
        // );

// Final y command: reverse shooter and dye wheels
        // driver.y().whileTrue(
        // new ReverseWheels(dyerotor, -0.9, turret, -0.6)
        // );

        // operator commands

        operator.x().onTrue(
            drivetrain.resetGyro()
        );

        operator.y().onTrue(
            turret.turnHood(-0.1)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
