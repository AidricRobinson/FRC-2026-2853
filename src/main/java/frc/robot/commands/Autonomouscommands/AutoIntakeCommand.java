package frc.robot.commands.Autonomouscommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GamepadConstants;

public class AutoIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private double durationInSeconds;
  private Timer timer;
  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, double durationInSeconds){
    this.intakeSubsystem = intakeSubsystem;
    this.durationInSeconds = durationInSeconds;
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
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return durationInSeconds <= timer.get();
  }
}
