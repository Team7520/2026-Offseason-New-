package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotorLeft;
    private final TalonFX intakeMotorRight;
    private final TalonFX extendMotor;
    private final TalonFX blockerMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
    double extendedPosition = -16.5; // placeholder value
    double retractedPosition = -5; // placeholder value
    private final double CURRENT_THRESHOLD = -20; // placeholder value

    public IntakeSubsystem() {
        intakeMotorLeft = new TalonFX(IntakeConstants.INTAKE_MOTOR_LEFT_ID);
        intakeMotorRight = new TalonFX(IntakeConstants.INTAKE_MOTOR_RIGHT_ID);
        extendMotor = new TalonFX(IntakeConstants.EXTEND_MOTOR_ID);
        blockerMotor = new TalonFX(IntakeConstants.BLOCKER_MOTOR_ID); // Placeholder IDs
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0; // placeholder value
        config.Slot0.kI = 0; // placeholder value
        config.Slot0.kD = 0; // placeholder value
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20; // placeholder value
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // placeholder value

        intakeMotorLeft.getConfigurator().apply(config);
        intakeMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        intakeMotorRight.getConfigurator().apply(config);
        intakeMotorRight.setNeutralMode(NeutralModeValue.Brake);

        config.Slot0.kP = 0; // placeholder value
        config.Slot0.kI = 0; // placeholder value
        config.Slot0.kD = 0; // placeholder value
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20; // placeholder value
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder value

        extendMotor.getConfigurator().apply(config);
        extendMotor.setNeutralMode(NeutralModeValue.Brake);

        config.Slot0.kP = 0; // placeholder value
        config.Slot0.kI = 0; // placeholder value
        config.Slot0.kD = 0; // placeholder value
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20; // placeholder value
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40; // placeholder value

        blockerMotor.getConfigurator().apply(config);
        blockerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void runIntake(double speed) {
        intakeMotorLeft.setControl(duty.withOutput(speed));
        intakeMotorRight.setControl(duty.withOutput(-speed));
    }

    public void extendSpin(double speed) {
        extendMotor.setControl(duty.withOutput(speed));
    }

    public void extend() {
        extendMotor.setControl(pivotPosReq.withPosition(extendedPosition));
    }

    public void retractWithSpeed() {
        extendMotor.setControl(pivotPosReq.withPosition(retractedPosition).withVelocity(0.05)); // placeholder value
    }

    public void retract() {
        extendMotor.setControl(pivotPosReq.withPosition(retractedPosition));
    }

    public void stopAll() {
        intakeMotorLeft.setControl(duty.withOutput(0));
        intakeMotorRight.setControl(duty.withOutput(0));
        extendMotor.setControl(duty.withOutput(0));
        blockerMotor.setControl(duty.withOutput(0));
    }

    public void resetPosition(double position) {
        extendMotor.setPosition(position);
    }

    public double getExtendedPosition() {
        return extendedPosition;
    }

    public void setCoast() {
        extendMotor.setControl(new CoastOut());
    }

    public void setNeutralforCurrent() {
        double currentDraw = extendMotor.getTorqueCurrent().getValueAsDouble();
        if (currentDraw <= CURRENT_THRESHOLD) {
            setCoast();
        }
    }

    public Command extendIntake() {
        return Commands.run(() -> extend(), this).until(() -> atTarget(extendedPosition));
        // .finallyDo(() -> setNeutral());
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> retract(), this);
    }

    public Command slowRetract() {
        return Commands.runOnce(() -> retractWithSpeed(), this);
    }

    public boolean atTarget(double position) {
        double current = extendMotor.getPosition().getValueAsDouble();
        double error = Math.abs(position - current);
        // System.out.print(error);
        return error < 0.1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", extendMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(
            "Intake deploy current", extendMotor.getTorqueCurrent().getValueAsDouble());
    }
}