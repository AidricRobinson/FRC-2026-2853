package frc.robot.commands.TestCommands.IntakeTestCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IntakeTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  private GenericHID controller;

public IntakeTestSetSpeed(IntakeSubsystem intakeSubsystem, GenericHID m_controller) {
    m_IntakeSubsystem = intakeSubsystem;
    controller = m_controller;
   
    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {
    m_IntakeSubsystem.setPoint(m_IntakeSubsystem.getIntakeTestSpeed());
}

  @Override
  public void execute() {
      m_IntakeSubsystem.updateError(); 
        m_IntakeSubsystem.setPower(
        m_IntakeSubsystem.getOutput() > 1 ? 1
        : m_IntakeSubsystem.getOutput() < 0 ? 0
        : m_IntakeSubsystem.getOutput()
        );
      // m_IntakeSubsystem.setPower(m_IntakeSubsystem.getIntakeTestSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.shutdown();
    m_IntakeSubsystem.reset();
  }

  @Override
  public boolean isFinished() {
    return !(controller.getRawButton(GamepadConstants.kXButtonPort));
  }
}

