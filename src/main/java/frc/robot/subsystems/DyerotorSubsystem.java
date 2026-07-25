package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyerotorSubsystem extends SubsystemBase {
    private final TalonFX inMotor;
    private final TalonFX upMotor;
    private final TalonFX spinMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public DyerotorSubsystem() {
        inMotor = new TalonFX(1);
        upMotor = new TalonFX(2);
        spinMotor = new TalonFX(3); // placeholder ids
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        inMotor.getConfigurator().apply(config);
        inMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        upMotor.getConfigurator().apply(config);
        upMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        spinMotor.getConfigurator().apply(config);
        spinMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void ballGoUp(double speed1, double speed2) {
        inMotor.setControl(duty.withOutput(speed1));
        upMotor.setControl(duty.withOutput(speed2));
    }

    public void spinMiddle(double speed) {
        spinMotor.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        inMotor.setControl(duty.withOutput(0));
        upMotor.setControl(duty.withOutput(0));
        spinMotor.setControl(duty.withOutput(0));
    }
}