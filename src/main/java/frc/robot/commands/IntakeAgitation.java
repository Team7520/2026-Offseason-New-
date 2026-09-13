package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAgitation extends Command {
  IntakeSubsystem intake;
  boolean agitateAgain = true;
  Timer timer = new Timer();

  public IntakeAgitation(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
/*    if (agitateAgain) {
      intake.agitate();
      agitateAgain = false;
    }
    if (timer.hasElapsed(0.5)) {
      agitateAgain = true;
      timer.reset();
    }
*/  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtend();
    timer.stop();
    timer.reset();
  }
}