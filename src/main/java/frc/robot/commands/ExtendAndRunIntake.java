package frc.robot.commands;

 import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndRunIntake extends Command {
  IntakeSubsystem intake;
  double speed;
  Timer timer = new Timer();

  public ExtendAndRunIntake(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    intake.extend();
    intake.runIntake(speed);
    if (timer.hasElapsed(0.7)) {
      intake.stopExtend();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopAll();
    timer.stop();
    timer.reset();
  }
}