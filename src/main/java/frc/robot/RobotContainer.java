// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import org.photonvision.EstimatedRobotPose;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.commands.ShootAndIndex;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem; 
import frc.robot.subsystems.DyerotorSubsystem; 
import frc.robot.subsystems.IntakeSubsystem;

import java.util.List;
import java.util.Optional;

public class RobotContainer {
    // Subsystems
    private final TurretSubsystem turret;
    private final DyerotorSubsystem dyerotor;
    private final IntakeSubsystem intake;
    //shooting
    final ShootAndIndex shootCommand;

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
        drivetrain::resetPose,
        drivetrain::followPath,
        true,          // mirror trajectory based on alliance
        drivetrain
    );

    public RobotContainer() {
        turret = new TurretSubsystem(drivetrain);
        dyerotor = new DyerotorSubsystem();
        intake = new IntakeSubsystem();

        shootCommand = new ShootAndIndex(dyerotor, intake, turret);

        // Configure the button bindings
        configureBindings();
    }

    public void setLocation(List<EstimatedRobotPose> visionEsts) {
        for (var est : visionEsts) {
            Pose2d pose = est.estimatedPose.toPose2d();
            drivetrain.addVisionMeasurement(pose, est.timestampSeconds);
        }
        System.out.println(drivetrain.getPose());
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
            shootCommand
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
        // drive.x().whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    AutoRoutine routine = autoFactory.newRoutine("MyAuto");
    AutoTrajectory traj = routine.trajectory("DoubleSwipeTop");

    routine.active().onTrue(
        traj.resetOdometry().andThen(traj.cmd())
    );

    traj.atTime("IntakeOn").onTrue(
        Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT TRIGGERED: IntakeOn!");
            intake.manualExtend(0.5);
        }, intake)
    );

    traj.atTime("IntakeOff").onTrue(
                Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT TRIGGERED: IntakeOff!");
            intake.stopAll();
        }, intake)
    );
    traj.atTime("ShootOn").onTrue(
    Commands.runOnce(() -> {
        System.out.println("CHOREO EVENT: Shoot ON");
       // shootCommand.schedule();
    })
);

traj.atTime("ShootOff").onTrue(
    Commands.runOnce(() -> {
        System.out.println("CHOREO EVENT: Shoot OFF");
        //shootCommand.cancel();
    })
);

    return routine.cmd();
}
}
