package frc.robot.commands.TestCommands.ShooterTestCommands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class ShooterTestShutdown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;
  private GenericHID controller;

   
  public ShooterTestShutdown(ShooterSubsystem shooterSubsystem, GenericHID m_controller) {
    m_ShooterSubsystem = shooterSubsystem;
    controller = m_controller;
   
    addRequirements(shooterSubsystem);
  }


  @Override
  public void initialize() {
    m_ShooterSubsystem.shutdown();
  }


  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return controller.getRawButton(GamepadConstants.kBButtonPort);
  }
}

