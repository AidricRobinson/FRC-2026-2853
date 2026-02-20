package frc.robot.commands.TestCommands.IndexorTestCommands;

import frc.robot.subsystems.IndexorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IndexorTestSpeedUp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexorSubsystem indexorSubsystem;
  private GenericHID controller;

   
  public IndexorTestSpeedUp(IndexorSubsystem indexorSubsystem, GenericHID m_controller) {
    this.indexorSubsystem = indexorSubsystem;
    controller = m_controller;
   
    addRequirements(indexorSubsystem);
  }


  @Override
  public void initialize() {
    indexorSubsystem.upSpeed();
      ///////////////////////////////////////////////
    /// ///////////////////////////////////////
    /// /ASDFHASDLFKJASDKLFjdsfljkasdjkl;fjklas:a/sdfhkljlas;diklfil;asdfjilasfd
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return controller.getPOV() == GamepadConstants.kDpadUp;
  }
}
