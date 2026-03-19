package frc.robot.commands.OperatorCommands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotUp extends Command{
private PivotSubsystem pivotSubsystem;
private GenericHID controller;
public ManualPivotUp(PivotSubsystem pivotSubsystem, GenericHID controller){
    this.pivotSubsystem = pivotSubsystem;
    this.controller = controller; 
}
@Override
public void initialize(){

}
@Override
public void execute(){
pivotSubsystem.setPower(.2);
}
@Override
public void end(boolean interrupted){
    pivotSubsystem.shutdown();
}
@Override
public boolean isFinished(){
    return !controller.getRawButton(YuanConstants.BottomLeft) || pivotSubsystem.getPivotEncoder() <= AutoConstants.kPivotUpPosition;
    // return !controller.getRawButton(GamepadConstants.kRightBumperPort) || pivotSubsystem.getPivotEncoder() <= AutoConstants.kPivotUpPosition;
}
}
