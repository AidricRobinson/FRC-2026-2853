package frc.robot.commands.TestCommands.IntakeTestCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IntakeTestSpeedUp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  private GenericHID controller;

   
  public IntakeTestSpeedUp(IntakeSubsystem intakeSubsystem, GenericHID m_controller) {
    m_IntakeSubsystem = intakeSubsystem;
    controller = m_controller;
   
    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    m_IntakeSubsystem.upSpeed();
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
    return controller.getRawButton(GamepadConstants.kYButtonPort);
  }
}
