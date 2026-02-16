package frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IntakeTestSetOutput extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private GenericHID controller;

public IntakeTestSetOutput(IntakeSubsystem intakeSubsystem, GenericHID m_controller) {
    this.intakeSubsystem = intakeSubsystem;
    controller = m_controller;
   
    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    // m_StorageSubsystem.setMotorTestSpeed();
  }

  @Override
  public void execute() {
    // intakeSubsystem.updateError(); 
    //     intakeSubsystem.setPower(
    //     intakeSubsystem.getOutput() > 1 ? 1
    //     : intakeSubsystem.getOutput() < 0 ? 0
    //     : intakeSubsystem.getOutput()
    //     );
    intakeSubsystem.setPower(intakeSubsystem.getIntakeTestOutput());
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return !(controller.getPOV() == GamepadConstants.kDpadLeft);
  }
}

