// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // private Timer disabledTimer;

    // private CANcoder absoluteEncoder;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

// @Override
//     public void robotInit()
//     {
//         absoluteEncoder = new CANcoder(/* Change this to the CAN ID of the CANcoder */ 13);
//         CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
//         cfg.apply(new CANcoderConfiguration());
//         MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
//         cfg.refresh(magnetSensorConfiguration);
//         cfg.apply(magnetSensorConfiguration
//                 .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
//         // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//         // autonomous chooser on the dashboard.

//         // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
//         // immediately when disabled, but then also let it be pushed more 
//         disabledTimer = new Timer();

//         if (isSimulation())
//         {
//             DriverStation.silenceJoystickConnectionWarning(true);
//         }
//     }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        // StatusSignal<Angle> angle = absoluteEncoder.getAbsolutePosition().waitForUpdate(0.1);

        // System.out.println("Absolute Encoder Angle (degrees): " + Units.rotationsToDegrees(angle.getValueAsDouble()));
        // // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // // commands, running already-scheduled commands, removing finished or interrupted commands,
        // // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
