package frc.robot.commands.Autonomouscommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class AutoShootCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem shooterSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final IndexorSubsystem indexorSubsystem;
    private double durationInSeconds;
    private double setPoint;
    private Timer timer;
    public AutoShootCommand(ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IndexorSubsystem indexorSubsystem, double durationInSecond, double setPoint){
        this.shooterSubsystem = shooterSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.durationInSeconds = durationInSecond;
        this.setPoint = setPoint;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
        shooterSubsystem.setPoint(setPoint);
        indexorSubsystem.setPoint(2500); // placeholder
        storageSubsystem.setPoint(2500); // placeholder
    }

    @Override
    public void execute() {
        shooterSubsystem.setPower(
            shooterSubsystem.getOutput()
        );
        if(timer.get() >= 2){
           indexorSubsystem.setPoint(
            indexorSubsystem.getOutput()
           );
           storageSubsystem.setPoint(
            storageSubsystem.getOutput()
           );
        }
  }

    @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shutdown();
    timer.stop();
    timer.reset();
  }

    @Override
  public boolean isFinished() {
    return durationInSeconds <= timer.get();
  }
}
