package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;

import java.lang.Math;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX topMotorLeft;
    private final TalonFX topMotorRight;
    private final TalonFX hoodMotor;
    private final TalonFX azimuthMotor;
    private final CANcoder encoder;
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public TurretSubsystem() {
        topMotorLeft = new TalonFX(TurretConstants.TOP_MOTOR_ID_LEFT);
        topMotorRight = new TalonFX(TurretConstants.TOP_MOTOR_ID_RIGHT);
        hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR_ID);
        azimuthMotor = new TalonFX(TurretConstants.AZIMUTH_MOTOR_ID);
        encoder = new CANcoder(55);

        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        topConfig.Slot0.kP = 0;
        topConfig.Slot0.kI = 0;
        topConfig.Slot0.kD = 0; // placeholder values
        topConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        topConfig.CurrentLimits.StatorCurrentLimit = 100;
        topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topConfig.CurrentLimits.SupplyCurrentLimit = 100; // placeholder values

        topMotorLeft.getConfigurator().apply(topConfig);
        topMotorLeft.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        topMotorRight.getConfigurator().apply(topConfig);
        topMotorRight.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        // Configure CANcoder
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0.08; // Adjust this value based on your magnet alignment
        encoder.getConfigurator().apply(cc_cfg);

        TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
        azimuthConfig.Slot0.kP = 100;
        azimuthConfig.Slot0.kI = 0;
        azimuthConfig.Slot0.kD = 0; // placeholder values

        azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        azimuthConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        azimuthConfig.Feedback.RotorToSensorRatio = TurretConstants.AZIMUTH_GEAR_RATIO;

        azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.StatorCurrentLimit = 60;
        azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        SoftwareLimitSwitchConfigs azimuthLimits = new SoftwareLimitSwitchConfigs();
        azimuthLimits.ForwardSoftLimitEnable = true;
        azimuthLimits.ForwardSoftLimitThreshold = 0.75;
        azimuthLimits.ReverseSoftLimitEnable = true;
        azimuthLimits.ReverseSoftLimitThreshold = -0.75;

        azimuthConfig.SoftwareLimitSwitch = azimuthLimits;
        azimuthMotor.getConfigurator().apply(azimuthConfig);
        azimuthMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 2;
        hoodConfig.Slot0.kI = 0;
        hoodConfig.Slot0.kD = 0; // placeholder values
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder values

        SoftwareLimitSwitchConfigs hoodLimits = new SoftwareLimitSwitchConfigs();
        hoodLimits.ForwardSoftLimitEnable = true;
        hoodLimits.ForwardSoftLimitThreshold = 3;
        hoodLimits.ReverseSoftLimitEnable = true;
        hoodLimits.ReverseSoftLimitThreshold = 0; 

        hoodConfig.SoftwareLimitSwitch = hoodLimits;

        hoodMotor.setPosition(0);

        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void turn(double speed) {
        azimuthMotor.setControl(duty.withOutput(speed));
    }

    public double getAzimuth() {
        return encoder.getPosition().getValueAsDouble();
    }

    public void setAzimuth(double angle) {
        Rotation2d normalized = Rotation2d.fromDegrees(angle);
        double rotations = normalized.getRotations();
        double curRotations = getAzimuth();

        double[] options = {rotations, rotations + 1, rotations - 1};
        double best = Double.POSITIVE_INFINITY;
        double bestChoice = curRotations;
        for (double d : options) {
            if (d > TurretConstants.AZIMUTH_LOWER_LIMIT && d < TurretConstants.AZIMUTH_UPPER_LIMIT && Math.abs(d - curRotations) < best) {
                best = Math.abs(d - curRotations);
                bestChoice = d;
            }
        }
        azimuthMotor.setControl(positionRequest.withPosition(bestChoice * TurretConstants.AZIMUTH_GEAR_RATIO));
    }

    public void hood(double speed) {
        hoodMotor.setControl(duty.withOutput(speed));
    }

    public void setHoodAngle(double angle) {
        //if (angle < TurretConstants.HOOD_MIN_ANGLE || angle > TurretConstants.HOOD_MAX_ANGLE) return;

        double normalized = (angle - TurretConstants.HOOD_MIN_ANGLE) / TurretConstants.HOOD_ANGLE_RANGE;
        double toRotations = TurretConstants.HOOD_MIN_ROTATION + normalized * TurretConstants.HOOD_ROTATION_RANGE;

        hoodMotor.setControl(positionRequest.withPosition(toRotations));
        System.out.println("running");

    }

    public void spinFlywheels(double speed) {
        topMotorRight.setControl(duty.withOutput(-speed));
        topMotorLeft.setControl(duty.withOutput(speed));
    }
        
    public Command shoot(double speed) {
        return Commands.run(() -> spinFlywheels(speed), this);
    }

    public Command turnHood(double speed) {
        return Commands.run(() -> hood(speed), this);
    }
    
    public void stopAll() {
        azimuthMotor.setControl(duty.withOutput(0));
        hoodMotor.setControl(duty.withOutput(0));
        topMotorRight.setControl(duty.withOutput(0));
        topMotorLeft.setControl(duty.withOutput(0));
    }

    @Override
    public void periodic() {
        System.out.println(azimuthMotor.getPosition().getValueAsDouble());
    }
}