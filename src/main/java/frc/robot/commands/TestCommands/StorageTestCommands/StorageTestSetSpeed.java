package frc.robot.commands.TestCommands.StorageTestCommands;

import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class StorageTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final StorageSubsystem m_StorageSubsystem;
  private GenericHID controller;

public StorageTestSetSpeed(StorageSubsystem storageSubsystem, GenericHID m_controller) {
    m_StorageSubsystem = storageSubsystem;
    controller = m_controller;
   
    addRequirements(storageSubsystem);
  }


  @Override
  public void initialize() {
    // m_StorageSubsystem.setMotorTestSpeed();
  }

  @Override
  public void execute() {
    m_StorageSubsystem.setPower(
      m_StorageSubsystem.getTestSpeed()
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_StorageSubsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return !(controller.getPOV() == GamepadConstants.kDpadLeft);
  }
}

