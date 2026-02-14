package frc.robot.commands.TestCommands.StorageTestCommands;

import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class StorageTestSpeedDown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final StorageSubsystem m_StorageSubsystem;
  private GenericHID controller;

   
  public StorageTestSpeedDown(StorageSubsystem storageSubsystem, GenericHID m_controller) {
    m_StorageSubsystem = storageSubsystem;
    controller = m_controller;
   
    addRequirements(storageSubsystem);
  }


  @Override
  public void initialize() {
    m_StorageSubsystem.downSpeed();
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
