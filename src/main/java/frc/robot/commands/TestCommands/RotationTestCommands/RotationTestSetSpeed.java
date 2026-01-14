package frc.robot.commands.TestCommands.RotationTestCommands;


import frc.robot.subsystems.RotationSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class RotationTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotationSubsystem rotationSubsystem;
  private GenericHID controller;

   
  public RotationTestSetSpeed(RotationSubsystem rotationSubsystem, GenericHID m_controller) {
    this.rotationSubsystem = rotationSubsystem;
    controller = m_controller;
   
    addRequirements(rotationSubsystem);
  }


  @Override
  public void initialize() {
    rotationSubsystem.setPower(0.03);
}

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    rotationSubsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return !controller.getRawButton(GamepadConstants.kRightBumperPort);
  }
}

