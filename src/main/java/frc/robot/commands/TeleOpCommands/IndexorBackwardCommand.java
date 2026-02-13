package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IndexorBackwardCommand extends Command{
    private IndexorSubsystem indexorSubsystem;
    private GenericHID controller;
    public IndexorBackwardCommand(IndexorSubsystem indexorSubsystem, GenericHID m_controller){
        this.indexorSubsystem = indexorSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        indexorSubsystem.setPower(-0.5);
    }
    public void end(boolean interupted){
        indexorSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);//pls change
    }
}
