package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.PivotSubsystem;

public class SlowPivotUp extends Command{
    private PivotSubsystem pivotSubsystem;
    private GenericHID controller;
    
    public SlowPivotUp(PivotSubsystem pivotSubsystem, GenericHID controller) {
        this.pivotSubsystem = pivotSubsystem;
        this.controller = controller;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pivotSubsystem.setPower(.1);
    }
    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.getPivotEncoder() <= AutoConstants.kPivotMiddle 
        || controller.getRawButton(YuanConstants.BottomLeft)
        || controller.getRawButton(YuanConstants.BottomRight);
    }
}
