package frc.robot.commands.TestCommands.IndexorTestCommands;

import frc.robot.subsystems.IndexorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IndexorTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexorSubsystem indexorSubsystem;
  private GenericHID controller;

public IndexorTestSetSpeed(IndexorSubsystem indexorSubsystem, GenericHID m_controller) {
    this.indexorSubsystem = indexorSubsystem;
    controller = m_controller;
   
    addRequirements(indexorSubsystem);
  }


  @Override
  public void initialize() {
    indexorSubsystem.setPoint(indexorSubsystem.getTestSpeed());
}

  @Override
  public void execute() {
      indexorSubsystem.updateError(); 
        indexorSubsystem.setPower(
        indexorSubsystem.getOutput() 
        );
      // indexorSubsystem.setPower(indexorSubsystem.getIntakeTestSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    indexorSubsystem.shutdown();
    indexorSubsystem.reset();
  }

  @Override
  public boolean isFinished() {
    return !(controller.getPOV() == GamepadConstants.kDpadLeft);
  }
}

