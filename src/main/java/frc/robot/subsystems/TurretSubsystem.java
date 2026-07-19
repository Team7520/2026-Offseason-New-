package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFX hoodMotor;
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public TurretSubsystem() {
        hoodMotor = new TalonFX(1);
        turnMotor = new TalonFX(2);
        topMotor = new TalonFX(3);
        bottomMotor = new TalonFX(4); // placeholder ids

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        turnMotor.getConfigurator().apply(config);
        turnMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values
        
        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        topMotor.getConfigurator().apply(config);
        topMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        bottomMotor.getConfigurator().apply(config);
        bottomMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void turn(double speed) {
        turnMotor.setControl(duty.withOutput(speed));
    }

    public void hood(double speed) {
        hoodMotor.setControl(duty.withOutput(speed));
    }

    public void top(double speed) {
        topMotor.setControl(duty.withOutput(speed));
    }

    public void bottom(double speed) {
        bottomMotor.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        turnMotor.setControl(duty.withOutput(0));
        hoodMotor.setControl(duty.withOutput(0));
        topMotor.setControl(duty.withOutput(0));
        bottomMotor.setControl(duty.withOutput(0));
    }
}