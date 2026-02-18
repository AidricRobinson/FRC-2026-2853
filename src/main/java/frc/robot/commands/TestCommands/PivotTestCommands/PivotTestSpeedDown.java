
package frc.robot.commands.TestCommands.PivotTestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotTestSpeedDown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem pivotSubsystem;
  private GenericHID controller;

   
  public PivotTestSpeedDown(PivotSubsystem pivotSubsystem, GenericHID m_controller) {
    this.pivotSubsystem = pivotSubsystem;
    controller = m_controller;
   
    addRequirements(pivotSubsystem);
  }


  @Override
  public void initialize() {
    pivotSubsystem.decreaseOutput();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return controller.getPOV() == GamepadConstants.kDpadDown;
  }
}

