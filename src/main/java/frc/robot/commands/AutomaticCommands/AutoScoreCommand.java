package frc.robot.commands.AutomaticCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoScoreCommand extends Command{
    private final LimelightSubsystem limelightSubsystem;
    private final IndexorSubsystem indexorSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();

    public AutoScoreCommand (ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, LimelightSubsystem limelightSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        indexorSubsystem.setPoint(4000);
        // shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();

        shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(limelightSubsystem.getTa()));

        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        if (timer.get() >= 1) {
            indexorSubsystem.updateError();
          indexorSubsystem.setPower(
            indexorSubsystem.getOutput() > 1 ? 1
            : indexorSubsystem.getOutput() < 0 ? 0
            : indexorSubsystem.getOutput()
          );
        }
    }
    @Override
    public void end (boolean interrupted) {
        shooterSubsystem.shutdown();
        indexorSubsystem.shutdown();

        shooterSubsystem.resetPID();
        indexorSubsystem.reset();
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= 6;
    }
}