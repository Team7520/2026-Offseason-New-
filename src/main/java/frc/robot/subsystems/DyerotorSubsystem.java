package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DyeConstants;

public class DyerotorSubsystem extends SubsystemBase {
    private final TalonFX wheelMotor;
    private final TalonFX dyeMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public DyerotorSubsystem() {
        wheelMotor = new TalonFX(DyeConstants.WHEEL_MOTOR_ID);
        dyeMotor = new TalonFX(DyeConstants.DYE_MOTOR_ID);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        wheelMotor.getConfigurator().apply(config);
        wheelMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        dyeMotor.getConfigurator().apply(config);
        dyeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void spinWheel(double speed) {
        wheelMotor.setControl(duty.withOutput(speed));
    }

    public void spinDye(double speed) {
        dyeMotor.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        wheelMotor.setControl(duty.withOutput(0));
        dyeMotor.setControl(duty.withOutput(0));
    }

    public Command intakeDye(double speed) {
        return Commands.run(() -> spinDye(speed), this);
    }

    public Command intakeWheel(double speed) {
        return Commands.run(() -> spinWheel(speed), this);
    }
}