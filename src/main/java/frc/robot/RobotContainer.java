// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.ReverseWheels;

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

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        turret = new TurretSubsystem();
        dyerotor = new DyerotorSubsystem();
        intake = new IntakeSubsystem();

        // Configure the button bindings
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // dyerotor.setDefaultCommand(
            // dyerotor.reverseDye(-0.1)
        // );

        // Testing the shooter
        driver.rightTrigger().whileTrue(
            turret.shoot(0.6)
        ).onFalse(
            new InstantCommand(() -> turret.stopAll(), turret) 
        );

        driver.leftTrigger().onTrue(
            new InstantCommand(() -> turret.setHoodAngle(35), turret)
        );

        driver.rightBumper().whileTrue(
            dyerotor.intakeDyeAndWheel(0.5, 0.6)
        ).onFalse(
            new InstantCommand(() -> dyerotor.stopAll()) 
        );

        driver.a().whileTrue(
            turret.turnHood(-0.1)
        ).onFalse(
            new InstantCommand(turret::stopAll)
        );

        driver.leftBumper().onTrue(
            intake.blockerToggle()
        ).onFalse(new InstantCommand(turret::stopAll));

        driver.x().whileTrue(
            Commands.run(() -> intake.manualExtend(0.5))
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );

        driver.y().whileTrue(
            Commands.run(() -> intake.manualExtend(-0.90))
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );


        // driver
        // .y().whileTrue(new ReverseWheels(dyerotor, -0.9, turret, -0.6));
/*
        driver
        .a()
        .whileTrue(
            intake
            .spinBlocker(0.1))
            .onFalse(new InstantCommand(() -> intake.stopAll()) );
*/

        driver
        .a()
        .whileTrue(
            turret
            .turnHood(-0.1))
            .onFalse(new InstantCommand(() -> turret.stopAll()) );

        driver
        .b()
        .whileTrue(
            turret
            .turnHood(0.1))
            .onFalse(new InstantCommand(() -> turret.stopAll()) );

        // driver
        // .y().whileTrue(new ReverseWheels(dyerotor, -0.9, turret, -0.6));
/*
        driver
        .a()
        .whileTrue(
            intake
            .spinBlocker(0.1))
            .onFalse(new InstantCommand(() -> intake.stopAll()) );
*/
/*
        driver
        .a()
        .whileTrue(
            intake
            .extendIntake())
            .onFalse(new InstantCommand(() -> intake.stopAll()) );
*/
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        // joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
