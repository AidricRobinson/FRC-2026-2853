package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveSlowModeCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private GenericHID controller;
    public SwerveSlowModeCommand (CommandSwerveDrivetrain swerve, GenericHID controller) {
        this.swerve = swerve;
        this.controller = controller;

    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        CommandSwerveDrivetrain.setSlowMode();
    }
    @Override
    public void end (boolean interrupted) {
        CommandSwerveDrivetrain.setNormalSpeed();
    }
    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kRightBumperPort);
    }
}
