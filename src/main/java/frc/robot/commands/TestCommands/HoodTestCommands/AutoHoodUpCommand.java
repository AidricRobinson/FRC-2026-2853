package frc.robot.commands.TestCommands.HoodTestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.HoodSubsystem;

public class AutoHoodUpCommand extends Command{
    private HoodSubsystem hoodSubsystem;
    public AutoHoodUpCommand (HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        hoodSubsystem.setPoint(AutoConstants.kHoodLaunchAngle);
    }

    @Override
    public void execute() {
        hoodSubsystem.updateError();
        hoodSubsystem.setPower(hoodSubsystem.getOutput());
    }

    @Override
    public void end(boolean isFinished) {
        hoodSubsystem.resetPID();
        hoodSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hoodSubsystem.getError()) < AutoConstants.kHoodTolerance;
    }
    
}
