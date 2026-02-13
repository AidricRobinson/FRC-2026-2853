package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.IndexorSubsystem;

public class IndexorForwardCommand extends Command{
   private IndexorSubsystem indexorSubsystem;
    private GenericHID controller;
    public IndexorForwardCommand(IndexorSubsystem indexorSubsystem, GenericHID m_controller){
        this.indexorSubsystem = indexorSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        indexorSubsystem.setPower(0.5);
    }
    public void end(boolean interupted){
        indexorSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kRightBumperPort); //pls change// no
    } 
}
