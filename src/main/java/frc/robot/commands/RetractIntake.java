package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends Command {
  IntakeSubsystem intake;
  double speed;

  public RetractIntake(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);

  }

  @Override
  public void execute() {
    intake.retract();
    intake.runIntake(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtend();
    intake.stopIntake();
  }
}