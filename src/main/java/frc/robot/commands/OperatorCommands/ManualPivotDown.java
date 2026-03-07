package frc.robot.commands.OperatorCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotDown extends Command{
private PivotSubsystem pivotSubsystem;
private GenericHID controller;
public ManualPivotDown(PivotSubsystem pivotSubsystem, GenericHID m_controller){
    this.pivotSubsystem = pivotSubsystem;
    m_controller = controller; 
}
@Override
public void initialize(){

}
@Override
public void execute(){
    pivotSubsystem.setPower(-.025);
}
@Override
public void end(boolean interrupted){
    pivotSubsystem.shutdown();
}
@Override
public boolean isFinished(){
    // return !controller.getRawButton(YuanConstants.BottomRight);
    return !controller.getRawButton(GamepadConstants.kXButtonPort);
}
}
