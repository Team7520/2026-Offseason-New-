package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.DyeConstants;;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX azimuthMotor;
    private final TalonFX hoodMotor;
    private final TalonFX topMotorLeft;
    private final TalonFX topMotorRight;
    private final TalonFX bottomMotor;
    private final TalonFX rotateMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public TurretSubsystem() {
        rotateMotor = new TalonFX(DyeConstants.DYE_ROTATE_MOTOR_ID);
        hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR_ID);
        azimuthMotor = new TalonFX(TurretConstants.AZIMUTH_MOTOR_ID);
        topMotorLeft = new TalonFX(TurretConstants.TOP_MOTOR_ID_LEFT);
        topMotorRight = new TalonFX(TurretConstants.TOP_MOTOR_ID_RIGHT);

        bottomMotor = new TalonFX(TurretConstants.BOTTOM_MOTOR_ID); // placeholder ids

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
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
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 100; // placeholder values

        topMotorLeft.getConfigurator().apply(config);
        topMotorLeft.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0; // placeholder values
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 100; // placeholder values

        topMotorRight.getConfigurator().apply(config);
        topMotorRight.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void turn(double speed) {
        azimuthMotor.setControl(duty.withOutput(speed));
    }

    public void hood(double speed) {
        hoodMotor.setControl(duty.withOutput(speed));
    }

    public void top(double speed) {
        topMotorRight.setControl(duty.withOutput(-speed));
        topMotorLeft.setControl(duty.withOutput(speed));
    }
    public Command shoot(double speed) {
        return Commands.run(() -> top(speed), this);
    }
    public Command turnHood(double speed) {
        return Commands.run(() -> hood(speed), this);
    }
    
    public void bottom(double speed) {
        bottomMotor.setControl(duty.withOutput(speed));
    }
    public Command turnBottom(double speed) {
        return Commands.run(() -> bottom(speed), this);
    }

    public void rotateDiThing(double speed1,double speed2) {
        rotateMotor.setControl(duty.withOutput(speed1));
        bottomMotor.setControl(duty.withOutput(speed2));
    }

    public Command turnRotate(double speed1,double speed2) {
        return Commands.run(() -> rotateDiThing(speed1,speed2), this);
    }

    public void stopAll() {
        azimuthMotor.setControl(duty.withOutput(0));
        hoodMotor.setControl(duty.withOutput(0));
        topMotorRight.setControl(duty.withOutput(0));
        topMotorLeft.setControl(duty.withOutput(0));
        bottomMotor.setControl(duty.withOutput(0));
        rotateMotor.setControl(duty.withOutput(0));
    }
}