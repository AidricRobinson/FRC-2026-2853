
package frc.robot.commands.TestCommands.ShooterTestCommands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class ShooterTestSetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;
  private GenericHID controller;

   
  public ShooterTestSetSpeed(ShooterSubsystem shooterSubsystem, GenericHID m_controller) {
    m_ShooterSubsystem = shooterSubsystem;
    controller = m_controller;
   
    addRequirements(shooterSubsystem);
  }


  @Override
  public void initialize() {
    // m_ShooterSubsystem.setPoint(m_ShooterSubsystem.getTestRPM());
}

  @Override
  public void execute() {
    // m_ShooterSubsystem.updateError(); 
    //     m_ShooterSubsystem.setPower(
    //     m_ShooterSubsystem.getOutput() > 1 ? 1
    //     : m_ShooterSubsystem.getOutput() < 0 ? 0
    //     : m_ShooterSubsystem.getOutput()
    //     );
    m_ShooterSubsystem.setPower(
      m_ShooterSubsystem.getShooterTestSpeed()
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.resetPID();
    m_ShooterSubsystem.shutdown();
  }

  @Override
  public boolean isFinished() {
    return !controller.getRawButton(GamepadConstants.kXButtonPort);
  }
}

