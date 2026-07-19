package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX extendMotor;
    private final TalonFX intakeMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(1);
        extendMotor = new TalonFX(2); // placeholder ids
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        extendMotor.getConfigurator().apply(config);
        extendMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void runIntake(double speed) {
        intakeMotor.setControl(duty.withOutput(speed));
    }

    public void extend(double speed) {
        extendMotor.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        intakeMotor.setControl(duty.withOutput(0));
        extendMotor.setControl(duty.withOutput(0));
    }
}