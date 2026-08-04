package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX azimuthMotor;
    private final TalonFX hoodMotor;
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public TurretSubsystem() {
        hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR_ID);
        azimuthMotor = new TalonFX(TurretConstants.AZIMUTH_MOTOR_ID);
        topMotor = new TalonFX(TurretConstants.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(TurretConstants.BOTTOM_MOTOR_ID); // placeholder ids

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        azimuthMotor.getConfigurator().apply(config);
        azimuthMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

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
        azimuthMotor.setControl(duty.withOutput(speed));
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
        azimuthMotor.setControl(duty.withOutput(0));
        hoodMotor.setControl(duty.withOutput(0));
        topMotor.setControl(duty.withOutput(0));
        bottomMotor.setControl(duty.withOutput(0));
    }
}