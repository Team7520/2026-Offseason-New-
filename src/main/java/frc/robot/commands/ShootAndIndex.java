package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DyerotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootAndIndex extends Command {
  DyerotorSubsystem dyerotor;
  TurretSubsystem turret;
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
    //turret.setHoodAngle(35);
//    turret.setAzimuth(0);
    //turret.spinFlywheels(0.6);
    if (timer.hasElapsed(0.3)) {
      dyerotor.spinWheel(0.9);
    }
    if (timer.hasElapsed(0.35)) {
      dyerotor.spinDye(0.6);
    }
  }

  @Override
  public void end(boolean interrupted) {
    dyerotor.spinWheel(0);
    dyerotor.spinDye(0);
    // turret.spinFlywheels(0);
    // turret.setHoodAngle(0);
    timer.stop();
    timer.reset();
  }
}