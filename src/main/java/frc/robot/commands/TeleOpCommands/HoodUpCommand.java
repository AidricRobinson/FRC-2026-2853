package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.HoodSubsystem;

public class HoodUpCommand extends Command {
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;

    public HoodUpCommand(HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;

        addRequirements(hoodSubsystem);
    }
    @Override 
    public void execute() {
        hoodSubsystem.setPower(.01);
    }
    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.shutdown();
    }
    @Override
    public boolean isFinished() {
        return !(controller.getPOV() == GamepadConstants.kDpadUp);
    }
}
