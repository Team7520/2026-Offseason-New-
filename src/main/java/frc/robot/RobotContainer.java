// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.IntArraySerializer;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;




public class RobotContainer {
    private double speedCutoff = 1;
    private double turnCutoff = 0.7;
    // Subsystems
    private final TurretSubsystem turret;
    private final DyerotorSubsystem dyerotor;
    private final IntakeSubsystem intake;

    // Auto Stuff
    private final InstantCommand shootCommand;
    private final Command dyerotorCommand;
    private final Command intakeCommand;




    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);


    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    

    private final AutoFactory autoFactory = new AutoFactory(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::followPath,
        true,          // mirror trajectory based on alliance
        drivetrain
    );
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public RobotContainer() {
        turret = new TurretSubsystem(drivetrain);
        dyerotor = new DyerotorSubsystem();
        intake = new IntakeSubsystem();

        dyerotorCommand = new ShootAndIndex(dyerotor, turret);
        shootCommand = new InstantCommand(() -> turret.shootCommand());
        intakeCommand = new ExtendAndRunIntake(intake, 0.9);

        // Configure the button bindings
        configureBindings();
        autoChooser.setDefaultOption("Double Swipe Top", buildAuto("DoubleSwipeTop"));
        autoChooser.addOption("Double Swipe Bottom", buildAuto("DoubleSwipeBottom"));
        autoChooser.addOption("8ball", buildAuto("Sneaky8Ball"));
        autoChooser.addOption("Do Nothing", Commands.none());

        SmartDashboard.putData("Auto Chooser", autoChooser);
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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed *speedCutoff) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed *speedCutoff) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate*turnCutoff) // Drive counterclockwise with negative X (left)
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

        driver.rightTrigger().whileTrue(
            turret.shootCommand()
        ).onTrue(
            new InstantCommand(
                () -> {
                  turnCutoff = turret.turnCutOff();
                  speedCutoff = turret.speedCutoff();
                })
        ).onFalse(
            new InstantCommand(
                () -> {
                  turnCutoff = 0.7;
                  speedCutoff = 1;
                })
        );
        
        driver.leftTrigger().whileTrue(
            new ExtendAndRunIntake(intake, 0.9)
        );

        driver.rightBumper().whileTrue(
            intake.spinRoller(-0.9)
        ).onFalse(
            new InstantCommand(() -> intake.stopIntake()) 
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
        driver.y().whileTrue(
            Commands.run(() -> intake.manualExtend(-0.9))
        ).onFalse(
            new InstantCommand(() -> intake.stopAll())
        );

        driver.rightStick().onTrue(
            intake.blockerToggle())
        .onFalse(new InstantCommand(turret::stopAll)
        );

        driver.x().whileTrue(
        drivetrain.applyRequest(() -> brake)
        );

        driver.y().whileTrue(
        new ReverseWheels(dyerotor, -0.3, turret, -0.5)
        );
/*
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
            new InstantCommand(() -> intake.stopIntake())
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
*/

        // operator commands

        operator.x().onTrue(
            drivetrain.resetGyro()
        );

        operator.y().whileTrue(
            turret.turnHood(-0.1).finallyDo(() -> turret.hood(0))
        );

        operator.b().onTrue(
            new InstantCommand(() -> dyerotor.toggleReverseDye())
        );

        operator.a().whileTrue(
            new InstantCommand(() -> intake.manualBlocker(0.1))
        ).onFalse(
            new InstantCommand(() -> intake.stopBlocker())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
        turret.setDefaultCommand(turret.autoAim());
    }

    private Command buildAuto(String trajectoryName) {
    AutoRoutine routine = autoFactory.newRoutine(trajectoryName);
    AutoTrajectory traj = routine.trajectory(trajectoryName);

    routine.active().onTrue(
        traj.resetOdometry().andThen(traj.cmd())
    );

    traj.atTime("IntakeOn").onTrue(
        Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT TRIGGERED: IntakeOn!");
            intakeCommand.schedule();
        })
    );

    traj.atTime("IntakeOff").onTrue(
        Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT TRIGGERED: IntakeOff!");
            intakeCommand.cancel();
            intake.stopAll();
        })
    );

    traj.atTime("ShootOn").onTrue(
        Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT: Shoot ON");
            dyerotorCommand.schedule();
            shootCommand.schedule();
        })
    );

    traj.atTime("ShootOff").onTrue(
        Commands.runOnce(() -> {
            System.out.println("CHOREO EVENT: Shoot OFF");
            dyerotorCommand.cancel();
            shootCommand.cancel();
        })
    );

    return routine.cmd();
}
public Command getAutonomousCommand() {
    Commands.waitSeconds(1.5);
    return autoChooser.getSelected();
}

}
