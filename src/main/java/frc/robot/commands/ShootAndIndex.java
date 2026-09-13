package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DyerotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootAndIndex extends Command {
  DyerotorSubsystem dyerotor;
  TurretSubsystem turret;
  double speed1;
  double speed2;
  Timer timer = new Timer();

  public ShootAndIndex(DyerotorSubsystem dyerotor, TurretSubsystem turret) {
    this.dyerotor = dyerotor;
    this.turret = turret;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    turret.setHoodAngle(35);
    turret.setAzimuth(0);
    turret.spinFlywheels(0.6);
    if (timer.hasElapsed(0.3)) {
      dyerotor.spinDyeAndWheel(0.65, 0.6);
    }
  }

  @Override
  public void end(boolean interrupted) {
    dyerotor.spinDyeAndWheel(0, 0);
    turret.spinFlywheels(0);
    timer.stop();
    timer.reset();
  }
}