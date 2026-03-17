package frc.robot.commands.TestCommands.HoodTestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.HoodSubsystem;

public class AutoHoodMiddleCommand extends Command{
    private HoodSubsystem hoodSubsystem;
    public AutoHoodMiddleCommand (HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        hoodSubsystem.setPoint(AutoConstants.kHoodFarArcAngle);
    }

    @Override
    public void execute() {
        hoodSubsystem.updateError();
        if (hoodSubsystem.getUpError() <= 0) {
            hoodSubsystem.setPower(hoodSubsystem.getUpOutput());
        }
        else {
            hoodSubsystem.setPower(hoodSubsystem.getDownOutput());
        }
        System.out.println("HOOD IS RUNNING HOOD IS RUNNING HOOD IS RUNNING");
        // System.out.println(hoodSubsystem.getOutput());

    }

    @Override
    public void end(boolean isFinished) {
        hoodSubsystem.resetPID();
        hoodSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hoodSubsystem.getUpError()) < AutoConstants.kHoodTolerance;
    }
    
}
