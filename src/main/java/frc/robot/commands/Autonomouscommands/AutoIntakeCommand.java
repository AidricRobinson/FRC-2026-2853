package frc.robot.commands.AutonomousCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GamepadConstants;

public class AutoIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;

  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem){
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    intakeSubsystem.setPoint(3000); // placeholder

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
    return false;
  }
}
