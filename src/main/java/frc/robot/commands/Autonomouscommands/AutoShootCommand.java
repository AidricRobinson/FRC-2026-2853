package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexorSubsystem indexorSubsystem;
  private double durationInSeconds;
  private double setPoint;
  private Timer timer;
  public AutoShootCommand(ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, double durationInSecond, double setPoint){
    this.shooterSubsystem = shooterSubsystem;
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
  }

  @Override
  public void execute() {
      shooterSubsystem.setPower(
          shooterSubsystem.getOutput()
      );
      if(timer.get() >= 2){
          indexorSubsystem.setPower(
          indexorSubsystem.getOutput()
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
