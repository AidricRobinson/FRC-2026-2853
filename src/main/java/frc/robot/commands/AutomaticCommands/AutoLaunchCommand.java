package frc.robot.commands.AutomaticCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoLaunchCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final IndexorSubsystem indexorSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private boolean shoot = false;
    private final Timer timer = new Timer();
    public AutoLaunchCommand (ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, HoodSubsystem hoodSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.hoodSubsystem = hoodSubsystem;

        addRequirements(shooterSubsystem, indexorSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();
        shooterSubsystem.setPoint(3500);
        indexorSubsystem.setPoint(4000);
        hoodSubsystem.setPoint(AutoConstants.kHoodLaunchAngle);
    }

    @Override
    public void execute() {
        if (timer.get() <= 1) {
            hoodSubsystem.setPower(
                hoodSubsystem.getUpOutput()
            );
            shooterSubsystem.setPower(
                shooterSubsystem.getOutput() > 1 ? 1
                : shooterSubsystem.getOutput() < 0 ? 0
                : shooterSubsystem.getOutput()   
            );
        }

        if (timer.get() >= 1 && timer.get() <= 5) {
            shooterSubsystem.setPower(
                shooterSubsystem.getOutput() > 1 ? 1
                : shooterSubsystem.getOutput() < 0 ? 0
                : shooterSubsystem.getOutput()   
            );
            indexorSubsystem.setPower(
                indexorSubsystem.getOutput() > 1 ? 1
                : indexorSubsystem.getOutput() < 0 ? 0
                : indexorSubsystem.getOutput()
            );
        }

        if (timer.get() >= 5) {
            hoodSubsystem.setPoint(
                AutoConstants.kHoodHighArcAngle
            );
            hoodSubsystem.setPower(
                hoodSubsystem.getDownOutput()
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.shutdown();
        shooterSubsystem.shutdown();
        indexorSubsystem.shutdown();

        hoodSubsystem.resetPID();
        shooterSubsystem.resetPID();
        indexorSubsystem.reset();

        timer.stop();
        timer.reset();
    }
    @Override
    public boolean isFinished(){
        return 6 <= timer.get();
    }
        
}
