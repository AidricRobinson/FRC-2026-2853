package frc.robot.commands.TestCommands.PivotTestCommands;

import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class PivotTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem pivotSubsystem;
  private GenericHID controller;

public PivotTestSetSpeed(PivotSubsystem pivotSubsystem, GenericHID m_controller) {
    this.pivotSubsystem = pivotSubsystem;
    controller = m_controller;
   
    addRequirements(pivotSubsystem);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    pivotSubsystem.setTestOutput();
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return !(controller.getPOV() == GamepadConstants.kDpadLeft);
  }
}

