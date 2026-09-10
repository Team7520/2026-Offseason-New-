// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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

    private final AutoFactory autoFactory = new AutoFactory(
        drivetrain::getPose,
        drivetrain::resetPoseForAuto,
        drivetrain::followPath,
        true,          // mirror trajectory based on alliance
        drivetrain
    );

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

        dyerotor.setDefaultCommand (
            dyerotor.intakeDyeAndWheel(-0.1, 0)
        );

        driver.rightTrigger().whileTrue(
            new ShootAndIndex(dyerotor, turret)
        );

        driver.leftTrigger().onTrue(
            new InstantCommand(() -> turret.setHoodAngle(35), turret)
        );

        driver.rightBumper().whileTrue(
            intake.spinRoller(-0.9)
        ).onFalse(
            new InstantCommand(() -> intake.stopAll()) 
        );

        driver.a().whileTrue(
            turret.turnHood(-0.1)
        ).onFalse(
            new InstantCommand(turret::stopAll)
        );
/*
        driver.leftBumper().onTrue(
            intake.blockerToggle()
        ).onFalse(new InstantCommand(turret::stopAll));
*/
        driver.leftBumper().whileTrue(
            Commands.run(() -> intake.manualExtend(0.5))
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );
/*
        driver.y().whileTrue(
            intake.spinRoller(0.9))
        .onFalse(
            new InstantCommand(() -> intake.stopAll())
        );
*/
/*
        driver.y().whileTrue(
            Commands.run(() -> intake.manualExtend(-0.9))
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );
*/

        driver.x().whileTrue(
            new InstantCommand(() -> turret.turn(0.2)))
        .onFalse(new InstantCommand(() -> turret.stopAll())
        );

        driver.y().whileTrue(
            new InstantCommand(() -> turret.turn(-0.2)))
        .onFalse(new InstantCommand(() -> turret.stopAll())
        );        

        driver.b().whileTrue(
            turret.turnHood(0.1))
        .onFalse(new InstantCommand(() -> turret.stopAll())
        );

// Final y command: reverse shooter and dye wheels
        // driver
        // .y().whileTrue(new ReverseWheels(dyerotor, -0.9, turret, -0.6));

// Final x command: x-cross wheels
        // drive.x().whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        AutoRoutine routine = autoFactory.newRoutine("MyAuto");
        AutoTrajectory traj = routine.trajectory("HighTideSigma");

        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));
        traj.atTime("IntakeOn").onTrue(new InstantCommand(() -> intake.manualExtend(0.5), intake));
        traj.atTime("IntakeOff").onTrue(new InstantCommand(() -> intake.stopAll()));

        return routine.cmd()
            .beforeStarting(() -> System.out.println("TRAJ CMD STARTING"))
            .finallyDo((interrupted) -> System.out.println("TRAJ CMD ENDED, interrupted=" + interrupted));
    }
}
