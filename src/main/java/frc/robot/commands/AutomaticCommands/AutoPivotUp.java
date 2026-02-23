package frc.robot.commands.AutomaticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotUp extends Command{
    private PivotSubsystem pivotSubsystem;

    public AutoPivotUp(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;   

        addRequirements(pivotSubsystem);
    } 

    @Override
    public void initialize() {
        pivotSubsystem.setPoint(AutoConstants.kPivotUpPosition);
    }

    @Override
    public void execute() {
        pivotSubsystem.setPower(
            pivotSubsystem.getOutput()
        );
    }

    @Override 
    public void end(boolean interrupted) {
        pivotSubsystem.shutdown();
        System.out.println("Pivot up finished");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivotSubsystem.getError()) <= AutoConstants.kPivotTolerance;
    }
    
}
