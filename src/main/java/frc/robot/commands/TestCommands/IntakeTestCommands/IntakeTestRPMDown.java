package frc.robot.commands.TestCommands.IntakeTestCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IntakeTestRPMDown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private GenericHID controller;

   
  public IntakeTestRPMDown(IntakeSubsystem intakeSubsystem, GenericHID m_controller) {
    this.intakeSubsystem = intakeSubsystem;
    controller = m_controller;
   
    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    intakeSubsystem.downRPM();
  }


  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (controller.getPOV() == GamepadConstants.kDpadDown);  
  }
}
