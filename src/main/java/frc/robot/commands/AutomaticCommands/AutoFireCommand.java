package frc.robot.commands.AutomaticCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoFireCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexorSubsystem indexorSubsystem;
    private LimelightSubsystem limelightSubsystem;
    public AutoFireCommand(ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, LimelightSubsystem limelightSubsystem, double ms) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        
        
        addRequirements(shooterSubsystem, indexorSubsystem);
    }
    @Override
    public void initialize() {
        shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(limelightSubsystem.getTa()));
        indexorSubsystem.setPoint(3000);

    }

    @Override
    public void execute() {
        shooterSubsystem.updateError(); 
        shooterSubsystem.setPower(
        shooterSubsystem.getOutput() > 1 ? 1
        : shooterSubsystem.getOutput() < 0 ? 0
        : shooterSubsystem.getOutput()
        );


        indexorSubsystem.updateError(); 
        indexorSubsystem.setPower(
        indexorSubsystem.getOutput() > 1 ? 1
        : indexorSubsystem.getOutput() < 0 ? 0
        : indexorSubsystem.getOutput()
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shutdown();
        shooterSubsystem.resetPID();
        
        indexorSubsystem.shutdown();
        indexorSubsystem.reset();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
