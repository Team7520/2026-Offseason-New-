package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DyeConstants;

// import java.lang.Math.abs;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX azimuthMotor;
    private final TalonFX hoodMotor;
    private final TalonFX topMotorLeft;
    private final TalonFX topMotorRight;
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final PositionDutyCycle pos = new PositionDutyCycle(0);

    public TurretSubsystem() {
        hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR_ID);
        azimuthMotor = new TalonFX(TurretConstants.AZIMUTH_MOTOR_ID);
        topMotorLeft = new TalonFX(TurretConstants.TOP_MOTOR_ID_LEFT);
        topMotorRight = new TalonFX(TurretConstants.TOP_MOTOR_ID_RIGHT);

        TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
        azimuthConfig.Slot0.kP = 0;
        azimuthConfig.Slot0.kI = 0;
        azimuthConfig.Slot0.kD = 0; // placeholder values
        azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.StatorCurrentLimit = 60;
        azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        azimuthMotor.getConfigurator().apply(azimuthConfig);
        azimuthMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 0;
        hoodConfig.Slot0.kI = 0;
        hoodConfig.Slot0.kD = 0; // placeholder values
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
        limits.ForwardSoftLimitEnable = true;
        limits.ForwardSoftLimitThreshold = 3;
        limits.ReverseSoftLimitEnable = true;
        limits.ReverseSoftLimitThreshold = 0; 

        hoodConfig.SoftwareLimitSwitch = limits;

        hoodMotor.setPosition(0);

        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration topLeftConfig = new TalonFXConfiguration();
        topLeftConfig.Slot0.kP = 0;
        topLeftConfig.Slot0.kI = 0;
        topLeftConfig.Slot0.kD = 0; // placeholder values
        topLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        topLeftConfig.CurrentLimits.StatorCurrentLimit = 100;
        topLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topLeftConfig.CurrentLimits.SupplyCurrentLimit = 100; // placeholder values

        topMotorLeft.getConfigurator().apply(topLeftConfig);
        topMotorLeft.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration topRightConfig = new TalonFXConfiguration();
        topRightConfig.Slot0.kP = 0;
        topRightConfig.Slot0.kI = 0;
        topRightConfig.Slot0.kD = 0; // placeholder values
        topRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        topRightConfig.CurrentLimits.StatorCurrentLimit = 100;
        topRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRightConfig.CurrentLimits.SupplyCurrentLimit = 100; // placeholder values

        topMotorRight.getConfigurator().apply(topRightConfig);
        topMotorRight.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void turn(double speed) {
        azimuthMotor.setControl(duty.withOutput(speed));
    }

    public double getAzimuth() {
        return azimuthMotor.getPosition().getValueAsDouble() / TurretConstants.AZIMUTH_GEAR_RATIO;
    }

    public void setAzimuth(double angle) {
        double rotations = angle / 360.0;
        double curRotations = getAzimuth();

        double[] options = {rotations, rotations + 1, rotations - 1};
        double best = 1000.0;
        double bestChoice = -1;
        for (double d : options) {
            if (d > TurretConstants.AZIMUTH_LOWER_LIMIT && d < TurretConstants.AZIMUTH_UPPER_LIMIT && Math.abs(d - curRotations) < best) {
                best = Math.abs(d - curRotations);
                bestChoice = d;
            }
        }
        azimuthMotor.setControl(pos.withPosition(bestChoice * TurretConstants.AZIMUTH_GEAR_RATIO));
    }

    public void hood(double speed) {
        hoodMotor.setControl(duty.withOutput(speed));
    }

    public void spinFlywheels(double speed) {
        topMotorRight.setControl(duty.withOutput(-speed));
        topMotorLeft.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        azimuthMotor.setControl(duty.withOutput(0));
        hoodMotor.setControl(duty.withOutput(0));
        topMotorRight.setControl(duty.withOutput(0));
        topMotorLeft.setControl(duty.withOutput(0));
    }
        
    public Command shoot(double speed) {
        return Commands.run(() -> spinFlywheels(speed), this);
    }

    public Command turnHood(double speed) {
        return Commands.run(() -> hood(speed), this);
    }
}