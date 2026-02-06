package frc.robot.commands.TestCommands.StorageTestCommands;

import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class StorageTestShutdown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final StorageSubsystem m_StorageSubsystem;
  private GenericHID controller;

   
  public StorageTestShutdown(StorageSubsystem storageSubsystem, GenericHID m_controller) {
    m_StorageSubsystem = storageSubsystem;
    controller = m_controller;
   
    addRequirements(storageSubsystem);
  }


  @Override
  public void initialize() {
    m_StorageSubsystem.testSpeedShutdown();
    m_StorageSubsystem.shutdown();
  }


  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (controller.getPOV() == GamepadConstants.kDpadRight);
  }
}
