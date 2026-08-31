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
    private final TalonFX rotateMotor;
    private final TalonFX wheelMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public DyerotorSubsystem() {
        rotateMotor = new TalonFX(DyeConstants.DYE_ROTATE_MOTOR_ID);
        wheelMotor = new TalonFX(DyeConstants.DYE_WHEEL_MOTOR_ID);
        
        TalonFXConfiguration rotateConfig = new TalonFXConfiguration();
        rotateConfig.Slot0.kP = 1;
        rotateConfig.Slot0.kI = 0;
        rotateConfig.Slot0.kD = 0; // placeholder values
        rotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotateConfig.CurrentLimits.SupplyCurrentLimit = 60;
        rotateConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rotateConfig.CurrentLimits.StatorCurrentLimit = 60; // placeholder values

        rotateMotor.getConfigurator().apply(rotateConfig);
        rotateMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
        wheelConfig.Slot0.kP = 1;
        wheelConfig.Slot0.kI = 0;
        wheelConfig.Slot0.kD = 0; // placeholder values
        wheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelConfig.CurrentLimits.SupplyCurrentLimit = 60;
        wheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelConfig.CurrentLimits.StatorCurrentLimit = 40; // placeholder values

        wheelMotor.getConfigurator().apply(wheelConfig);
        wheelMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }
 
    public void spinDye(double speed) {
        rotateMotor.setControl(duty.withOutput(speed));
    }

    public void spinWheel(double speed) {
        wheelMotor.setControl(duty.withOutput(-speed));
    }

    public void spinDyeAndWheel(double speed1, double speed2) {
        rotateMotor.setControl(duty.withOutput(speed1));
        wheelMotor.setControl(duty.withOutput(-speed2));
    }

    public Command intakeDyeAndWheel(double speed1, double speed2) {
        return Commands.run(() -> spinDyeAndWheel(speed1, speed2), this);
    }

    public void stopAll() {
        rotateMotor.setControl(duty.withOutput(0));
        wheelMotor.setControl(duty.withOutput(0));
    }
}