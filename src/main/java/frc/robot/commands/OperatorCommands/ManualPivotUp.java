package frc.robot.commands.OperatorCommands;
import frc.robot.Constants.GamepadConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotUp extends Command{
private PivotSubsystem pivotSubsystem;
private GenericHID controller;
public ManualPivotUp(PivotSubsystem pivotSubsystem, GenericHID m_controller){
    this.pivotSubsystem = pivotSubsystem;
    m_controller = controller; 
}
@Override
public void initialize(){

}
@Override
public void execute(){
pivotSubsystem.setPower(.025);
}
@Override
public void end(boolean interrupted){
    pivotSubsystem.shutdown();
}
@Override
public boolean isFinished(){
    // return !controller.getRawButton(YuanConstants.BottomLeft);
    return !controller.getRawButton(GamepadConstants.kYButtonPort);
}
}
