package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndRunIntake extends Command {
  IntakeSubsystem intake;
  double speed;

  public ExtendAndRunIntake(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void execute() {
    intake.extend();
    intake.runIntake(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopAll();
  }
}