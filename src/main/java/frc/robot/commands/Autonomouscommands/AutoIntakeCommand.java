package frc.robot.commands.AutonomousCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private Timer timer;

  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem){
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    intakeSubsystem.setPoint(3000); // placeholder
    timer.start();

  }
  @Override
  public void execute() {
    intakeSubsystem.setPower(
        intakeSubsystem.getOutput()
    );
  }
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.shutdown();
  }
  @Override
  public boolean isFinished() {
    return timer.get() >= 5;
  }
}
