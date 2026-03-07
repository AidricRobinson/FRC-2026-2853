package frc.robot.commands.TestCommands.HangCommands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.HangSubsystem;

public class HangPivotUpCommand extends Command{
    private HangSubsystem hangSubsystem;
    private GenericHID controller;

    public HangPivotUpCommand (HangSubsystem hangSubsystem, GenericHID controller) {
        this.hangSubsystem = hangSubsystem;
        this.controller = controller;

        addRequirements(hangSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hangSubsystem.setPivotPower(.2);
    }

    @Override
    public void end(boolean interrupted) {
        hangSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !(controller.getPOV() == GamepadConstants.kDpadRight);
    }
}
