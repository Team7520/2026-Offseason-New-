package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DyerotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ReverseWheels extends Command {
  DyerotorSubsystem dyerotor;
  TurretSubsystem turret;
  double speed1;
  double speed2;

  public ReverseWheels(DyerotorSubsystem dyerotor, double speed1, TurretSubsystem turret, double speed2) {
    this.dyerotor = dyerotor;
    this.turret = turret;
    this.speed1 = speed1;
    this.speed2 = speed2;
  }

  @Override
  public void execute() {
    dyerotor.spinWheel(speed1);
    turret.spinFlywheels(speed2);
  }

  @Override
  public void end(boolean interrupted) {
    dyerotor.spinWheel(0);
    turret.spinFlywheels(0);
  }
}