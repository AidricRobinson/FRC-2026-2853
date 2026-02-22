package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotDown extends Command{
    private PivotSubsystem pivotSubsystem;

    public AutoPivotDown(PivotSubsystem pivotSubsystem) {
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
            AutoConstants.kPivotDownSpeed
        );
    }

    @Override 
    public void end(boolean interrupted) {
        pivotSubsystem.shutdown();
        System.out.println("Pivot down finished");
    }

    @Override
    public boolean isFinished() {
        // return pivotSubsystem.isBeamBroken();
        return pivotSubsystem.isPressed();
    }
    
}
