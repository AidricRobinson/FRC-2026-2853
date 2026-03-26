package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;

public class IndexorBackwardCommand extends Command{
    private IndexorSubsystem indexorSubsystem;
    private GenericHID controller;
    public IndexorBackwardCommand(IndexorSubsystem indexorSubsystem, GenericHID m_controller){
        this.indexorSubsystem = indexorSubsystem;
        controller = m_controller;

        addRequirements(indexorSubsystem);
    }
    public void initialize(){
        indexorSubsystem.setPoint(-5000);
    }
    public void execute(){
        indexorSubsystem.updateError();
        indexorSubsystem.setPower(
            indexorSubsystem.getOutput() 
        );
        // indexorSubsystem.setPower(-1);
        
    }
    public void end(boolean interupted){
        indexorSubsystem.shutdown();
    }
    public boolean isFinished(){
        // return !controller.getRawButton(GamepadConstants.kLeftBumperPort);//pls change
        return !controller.getRawButton(YuanConstants.SideBottom);
    }
}
