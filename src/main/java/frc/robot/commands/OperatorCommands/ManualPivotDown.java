package frc.robot.commands.OperatorCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotDown extends Command{
private PivotSubsystem pivotSubsystem;
private IntakeSubsystem intakeSubsystem;
private GenericHID controller;
public ManualPivotDown(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem, GenericHID controller){
    this.pivotSubsystem = pivotSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.controller = controller; 

    addRequirements(pivotSubsystem, intakeSubsystem);
}
@Override
public void initialize(){

}
@Override
public void execute(){
    if (pivotSubsystem.getPivotEncoder() <= AutoConstants.kPivotDownPosition) {
        pivotSubsystem.setPower(-.2);
        intakeSubsystem.setPower(-0.2); 
    }
    if ((pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition)) {
        pivotSubsystem.setRightPivotPower(-0.05);
        pivotSubsystem.setLeftPivotPower(0);
    }
    
}
@Override
public void end(boolean interrupted){
    pivotSubsystem.shutdown();
    intakeSubsystem.shutdown(); 
}
@Override
public boolean isFinished(){
    return !controller.getRawButton(YuanConstants.BottomRight) || ((pivotSubsystem.getCurrent() >= AutoConstants.kCurrentLimit) && (pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition)) ;
    // return !controller.getRawButton(GamepadConstants.kLeftBumperPort) || pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition;
}
}
