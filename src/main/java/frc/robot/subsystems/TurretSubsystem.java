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
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;
import frc.robot.generated.TunerConstants;

import java.lang.Math;
import java.util.NoSuchElementException;

public class TurretSubsystem extends SubsystemBase {

    public enum RobotZone {
        SHOOTING,
        RED_FEEDING_OUTPOST,
        BLUE_FEEDING_OUTPOST,
        RED_FEEDING_DEPOT,
        BLUE_FEEDING_DEPOT,
        UNDER_FAR_TRENCH,
        NO_ALLIANCE
    }

    StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();

    private final TalonFX topMotorLeft;
    private final TalonFX topMotorRight;
    private final TalonFX hoodMotor;
    private final TalonFX azimuthMotor;
    private final CANcoder encoder;
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltRequest = new VelocityVoltage(0);

    CommandSwerveDrivetrain drive;

    Alliance currentAlliance = null;
    boolean availableAlliance = false;

    private boolean setWheels = false;
    private boolean hoodAdjust = false;
    private boolean feederToggle = false;
    private boolean override = false;

    private double goalPoseX;
    private double goalPoseY;
    private double feedOutpostPoseX;
    private double feedOutpostPoseY;
    private double feedDepotPoseX;
    private double feedDepotPoseY;
    private double updatingHoodPos = 0;
    private boolean far = false;
    double updatingCurrentDist = 0;
    private Alliance alliance = null;
    private Pose2d goal;
    private Pose2d feedOutpostPose;
    private Pose2d feedDepotPose;

    public TurretSubsystem(CommandSwerveDrivetrain drive) {
        this.drive = drive;

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
        azimuthConfig.Slot0.kP = 20;
        azimuthConfig.Slot0.kI = 0;
        azimuthConfig.Slot0.kD = 0; // placeholder values

        azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        azimuthConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        azimuthConfig.Feedback.RotorToSensorRatio = TurretConstants.AZIMUTH_GEAR_RATIO;

        azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.StatorCurrentLimit = 40;
        azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.SupplyCurrentLimit = 20; // placeholder values

        SoftwareLimitSwitchConfigs azimuthLimits = new SoftwareLimitSwitchConfigs();
        azimuthLimits.ForwardSoftLimitEnable = true;
        azimuthLimits.ForwardSoftLimitThreshold = 0.72;
        azimuthLimits.ReverseSoftLimitEnable = true;
        azimuthLimits.ReverseSoftLimitThreshold = -0.72;

        azimuthConfig.SoftwareLimitSwitch = azimuthLimits;
        azimuthConfig.Feedback.SensorToMechanismRatio = 1;
        azimuthMotor.getConfigurator().apply(azimuthConfig);
        azimuthMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 1;
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

    public void setTurretAzimuth(Rotation2d targetAngle) {
        double target = -targetAngle.getRotations();
        double clampedTarget = optimizeTurretPosition(target);
        SmartDashboard.putNumber("Clamped Target", clampedTarget);
        azimuthMotor.setControl(positionRequest.withPosition(clampedTarget));
    }

    private double optimizeTurretPosition(double targetPosition) {
        double forwardLimit = 0.5;
        double reverseLimit = -0.5;

        if (getRobotZone() == RobotZone.SHOOTING) {
            forwardLimit = TurretConstants.TURRET_FORWARD_LIMIT;
            reverseLimit = TurretConstants.TURRET_REVERSE_LIMIT;
        }

        // Get current position
        double currentPosition = azimuthMotor.getPosition().getValueAsDouble();

        // Clamp target to the valid range [-0.5, 0.5]
        double clampedTarget = MathUtil.clamp(targetPosition, -0.5, 0.5);

        // Try the target position as-is and with ±1 rotation offset
        double[] candidates = {clampedTarget, clampedTarget + 1.0, clampedTarget - 1.0};

        // Find the candidate that is within bounds and requires shortest distance
        double bestPosition = clampedTarget;
        double shortestDistance = Double.MAX_VALUE;

        for (double candidate : candidates) {
            // Check if within dynamic limits
            if (candidate >= reverseLimit && candidate <= forwardLimit) {
                // Calculate distance from current position
                double distance = Math.abs(candidate - currentPosition);
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    bestPosition = candidate;
                }
            }
        }

    return bestPosition;
    }

    /*
    * @param robotPose
    * @param goalPose
    * @return an Rotation2D from −180°, 180°
    */
    public Rotation2d calculateTurretAzimuth(Pose2d robotPose, Pose2d goalPose) {
        Transform2d robotToTurret = new Transform2d(new Translation2d(), new Rotation2d());
        Pose2d turretPose = robotPose.transformBy(robotToTurret);
        Translation2d turretToGoal = goalPose.getTranslation().minus(turretPose.getTranslation());
        Rotation2d fieldAngle = turretToGoal.getAngle();
        turretPosePublisher.set(turretPose);

        return fieldAngle.minus(robotPose.getRotation()).plus(new Rotation2d(0.77 * Math.PI)); // Adjust for turret offset angle
    }


    public Pose2d predictFuturePose(Pose2d robotPose, double timeOfFlight, double odometryLatency) {
        ChassisSpeeds currentSpeed = drive.getFieldRelativeSpeeds();
        SmartDashboard.putNumber("Current Speed VX", currentSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("Current Speed VY", currentSpeed.vyMetersPerSecond);
        return new Pose2d(
            robotPose.getX() + currentSpeed.vxMetersPerSecond * (odometryLatency + timeOfFlight),
            robotPose.getY() + currentSpeed.vyMetersPerSecond * (timeOfFlight + odometryLatency),
            robotPose.getRotation().plus(new Rotation2d(currentSpeed.omegaRadiansPerSecond * odometryLatency))
        );
    }

    public RobotZone getRobotZone() {
        if (!availableAlliance) {
            return RobotZone.NO_ALLIANCE;
        }
        Pose2d turretPose = getTurretPose();
        double xPosition = turretPose.getX();
        double yPosition = turretPose.getY();

        if (alliance == Alliance.Red) {
            // RED ALLIANCE
            if (xPosition <= 6 && xPosition >= 3.7) {
                return RobotZone.UNDER_FAR_TRENCH;
            } else if (xPosition <= 11) {
                if (yPosition >= UniverseConstants.fieldWidthMidpoint) {
                    return RobotZone.RED_FEEDING_OUTPOST;
                } else {
                    return RobotZone.RED_FEEDING_DEPOT;
                }
            } else {
                return RobotZone.SHOOTING;
            }
        } else {
            // BLUE ALLIANCE
            if (xPosition <= 12.7 && xPosition >= 11) {
                return RobotZone.UNDER_FAR_TRENCH;
            } else if (xPosition >= 6) {
                if (yPosition >= UniverseConstants.fieldWidthMidpoint) {
                    return RobotZone.BLUE_FEEDING_OUTPOST;
                } else {
                    return RobotZone.BLUE_FEEDING_DEPOT;
                }
            } else {
                return RobotZone.SHOOTING;
            }
        }
    }

    public Command autoAim() {
        return Commands.run(() -> {
                if (!override) {
                    RobotZone zone = getRobotZone();
                    SmartDashboard.putString("Robot Zone", zone.toString());
                    if (zone == RobotZone.NO_ALLIANCE) {
                        return;
                    }
                switch (zone) {
                    case SHOOTING:
                    case RED_FEEDING_DEPOT:
                    case BLUE_FEEDING_DEPOT:
                    case RED_FEEDING_OUTPOST:
                    case BLUE_FEEDING_OUTPOST:
                    {
                        Pose2d robotPose = drive.getPose();
                        Pose2d targetPose;
                        switch (zone) {
                            case SHOOTING:
                                targetPose = goal;
                                break;
                            case BLUE_FEEDING_DEPOT:
                            case RED_FEEDING_DEPOT:
                                targetPose = feedDepotPose;
                                break;
                            case BLUE_FEEDING_OUTPOST:
                            case RED_FEEDING_OUTPOST:
                                targetPose = feedOutpostPose;
                                break;
                            default:
                                targetPose = goal;
                                break;
                        }
                        far = false;
                        if (availableAlliance) {
                            if (alliance == Alliance.Red) {
                                if (robotPose.getX() <= 6) {
                                    far = true;
                                }

                            } else {
                                if (robotPose.getX() >= 11) {
                                    far = true;
                                }
                            }
                        }
                        double dist = getDistance(robotPose, targetPose);
                        Pose2d currentPose = robotPose;
                        double currentDist = dist;
                        // 0.13
                        double odometryLatency = 0.1;

                        double flightTime = 0.125 * currentDist + 0.665;
                        currentPose = drive.getPose();
                        currentPose = predictFuturePose(robotPose, flightTime, odometryLatency);
                        updatingCurrentDist = getDistance(currentPose, targetPose);

                        updatingHoodPos = getHoodFromDistance(updatingCurrentDist, far);
                        Rotation2d turretAngle = calculateTurretAzimuth(currentPose, targetPose);
                        setTurretAzimuth(turretAngle);

                        if (setWheels) {
                            setFlywheelVelocity(getSpeedFromDistance(updatingCurrentDist, far));
                        } else {
                            stopFlywheels();
                        }

                        if (hoodAdjust && !override) {
                            setHoodAngle(updatingHoodPos);
                        } else {
                            setHoodAngle(1);
                        }

                        SmartDashboard.putNumber("Distance to target", currentDist);
                        SmartDashboard.putNumber("TURRET ROT", turretAngle.getRotations());
                        SmartDashboard.putNumber("TURRET DEG", turretAngle.getDegrees());
                        break;
                    }
                    case UNDER_FAR_TRENCH:
                        {
                            setHoodAngle(1);
                            break;
                        }
                    case NO_ALLIANCE:
                        {
                            System.out.println("Did nothing, alliance not selected yet!");
                            break;
                        }
                }
            }
        }, this);
    }

    public Command shootCommand() {
    return Commands.defer(
        () -> {
          if (override) {
            return Commands.parallel(
                    Commands.run(
                        () -> {
                          setWheels = true;
                          hoodAdjust = false;
                        })
                    )
                .finallyDo(
                    () -> {
                      setWheels = false;
                      stopFlywheels();
                    });
          }
          RobotZone zone = getRobotZone();
          if (zone == RobotZone.UNDER_FAR_TRENCH) {
            return Commands.startEnd(
                () -> {
                  hoodAdjust = false;
                },
                () -> {});
          } else {
            return Commands.run(() -> {
                setWheels = true;
                hoodAdjust = true;
                System.out.println("abcdefghijklmnop");
            })
            .finallyDo(() -> {
                setWheels = false;
                hoodAdjust = false;
                System.out.println("1234567890");
            });
          }
        },
        java.util.Set.of());
  }

    public double getDistance(Pose2d turretPose, Pose2d goalPose) {
        return turretPose.getTranslation().getDistance(goalPose.getTranslation());
    }

    public Pose2d getTurretPose() {
        return drive.getPose();
    }

    public double getHoodFromDistance(double distance, boolean far) {
        if (far) {
            distance += 3;
        }
        double scaleFactor = 0.5833;

        double hoodPos = (distance - 2.0) * scaleFactor;
        return hoodPos;
    }

    public double getSpeedFromDistance(double distance, boolean far) {
        if (far) {
            distance += 3;
        }
        double b = 23.67;
     // 3.35
        double rpsPerDistance = 3.3;
        double speed = rpsPerDistance * distance + b;

        // for testing
        // double speed = 36.0;

        if (speed > 75.0) {
            speed = 75.0;
        }
        return speed;
    }

    public void turn(double speed) {
        azimuthMotor.setControl(duty.withOutput(speed));
    }

    public double getAzimuth() {
        return encoder.getPosition().getValueAsDouble();
    }
/*
    public void setTurretAngle(double targetDegrees) {
        double currentRotations = turretMotor.getPosition().getValueAsDouble();
        double targetRotations = targetDegrees / 360.0;

        // find the equivalent target closest to current position (shortest path)
        double delta = targetRotations - currentRotations;
        delta -= Math.round(delta); // wraps delta into [-0.5, 0.5) rotations
        double setpoint = currentRotations + delta;

        turretMotor.setControl(new MotionMagicVoltage(setpoint));
    }
*/
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
        azimuthMotor.setControl(positionRequest.withPosition(bestChoice));
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

    public void setFlywheelVelocity(double rps) {
        SmartDashboard.putNumber("RPS target", rps);
        topMotorLeft.setControl(velocityVoltRequest.withVelocity(-rps).withEnableFOC(true));
        topMotorRight.setControl(velocityVoltRequest.withVelocity(rps).withEnableFOC(true));
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

    public void stopFlywheels() {
        topMotorRight.setControl(duty.withOutput(0));
        topMotorLeft.setControl(duty.withOutput(0));
    }

    public void override() {
        if (!override) {
            override = true;
        } else {
            override = false;
        }
    }

    public boolean atTarget(double position) {
        double current = azimuthMotor.getPosition().getValueAsDouble();
        double error = Math.abs(position - current);
        // System.out.print(error);
        return error < 0.01;
    }


    @Override
    public void periodic() {
//        System.out.println("Hood Position: " + hoodMotor.getPosition().getValueAsDouble());
//        System.out.println("Azimuth Motor Position: " + azimuthMotor.getPosition().getValueAsDouble());
//        System.out.println("Encoder Motor Position: " + encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Position", hoodMotor.getPosition().getValueAsDouble());
        if (!availableAlliance) {
            try {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    goalPoseX = UniverseConstants.redGoalPose.getX();
                    goalPoseY = UniverseConstants.redGoalPose.getY();
                    feedOutpostPoseX = UniverseConstants.redOutpostFeedX;
                    feedOutpostPoseY = UniverseConstants.redOutpostFeedY;
                    feedDepotPoseX = UniverseConstants.redDepotFeedX;
                    feedDepotPoseY = UniverseConstants.redDepotFeedY;
                    availableAlliance = true;
                    alliance = Alliance.Red;
                } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    goalPoseX = UniverseConstants.blueGoalPose.getX();
                    goalPoseY = UniverseConstants.blueGoalPose.getY();
                    feedOutpostPoseX = UniverseConstants.blueOutpostFeedX;
                    feedOutpostPoseY = UniverseConstants.blueOutpostFeedY;
                    feedDepotPoseX = UniverseConstants.blueDepotFeedX;
                    feedDepotPoseY = UniverseConstants.blueDepotFeedY;
                    availableAlliance = true;
                    alliance = Alliance.Blue;
                }
                goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());
                feedOutpostPose = new Pose2d(feedOutpostPoseX, feedOutpostPoseY, new Rotation2d());
                feedDepotPose = new Pose2d(feedDepotPoseX, feedDepotPoseY, new Rotation2d());
            } catch (NoSuchElementException nsee) {
                System.out.println("No available alliance!");
            }
        }
    }
}