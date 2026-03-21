package frc.robot.commands.AutomaticCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexorSubsystem indexorSubsystem;
  private double durationInSeconds;
  private double setPoint;
  private final Timer timer = new Timer();
  public AutoShootCommand(ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, double durationInSecond, double setPoint){
    this.shooterSubsystem = shooterSubsystem;
    this.indexorSubsystem = indexorSubsystem;
    this.durationInSeconds = durationInSecond;
    this.setPoint = setPoint;
    // timer = new Timer(); 
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize(){
      timer.start();

      shooterSubsystem.setPoint(setPoint);
  
      indexorSubsystem.setPoint(4000); // placeholder
  }

  @Override
  public void execute() {
      shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );
      if(timer.get() >= 1){
          indexorSubsystem.updateError();
          indexorSubsystem.setPower(
            indexorSubsystem.getOutput() > 1 ? 1
            : indexorSubsystem.getOutput() < 0 ? 0
            : indexorSubsystem.getOutput()
          );
      }
  }
  
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shutdown();
    shooterSubsystem.resetPID();
    indexorSubsystem.shutdown();
    indexorSubsystem.reset();
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return durationInSeconds <= timer.get();
    // return true;
  }
}
