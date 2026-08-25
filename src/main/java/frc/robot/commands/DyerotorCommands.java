package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DyerotorSubsystem;

public class DyerotorCommands extends Command {

  DyerotorSubsystem dyerotor;
  double speed;

  public DyerotorCommands(DyerotorSubsystem dyerotor, double speed) {
    this.dyerotor = dyerotor;
    this.speed = speed;
  }

  @Override
  public void execute() {
    dyerotor.spinDye(speed);
    dyerotor.spinWheel(speed);
  }

  @Override
  public void end(boolean interrupted) {
    dyerotor.stopAll();
  }
}