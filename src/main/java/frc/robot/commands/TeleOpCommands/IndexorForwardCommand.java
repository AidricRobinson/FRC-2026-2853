package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.IndexorSubsystem;

public class IndexorForwardCommand extends Command{
   private IndexorSubsystem indexorSubsystem;
    private GenericHID controller;
    public IndexorForwardCommand(IndexorSubsystem indexorSubsystem, GenericHID m_controller){
        this.indexorSubsystem = indexorSubsystem;
        controller = m_controller;

        addRequirements(indexorSubsystem);
    }
    public void initialize(){
        indexorSubsystem.setPoint(3500);
    }
    public void execute(){
        indexorSubsystem.updateError();
        indexorSubsystem.setPower(
            indexorSubsystem.getOutput() > 1 ? 1
            : indexorSubsystem.getOutput() < 0 ? 0.75
            : indexorSubsystem.getOutput()
        );
        // indexorSubsystem.setPower(1);
    }
    public void end(boolean interupted){
        indexorSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(YuanConstants.SideTop); 
    } 
}
