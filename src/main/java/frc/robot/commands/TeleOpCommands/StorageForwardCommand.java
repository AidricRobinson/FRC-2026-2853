package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.StorageSubsystem;

public class StorageForwardCommand extends Command{
   private StorageSubsystem m_conveyorSubsystem;
    private GenericHID controller;
    public StorageForwardCommand(StorageSubsystem conveyorSubsystem, GenericHID m_controller){
        m_conveyorSubsystem = conveyorSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        m_conveyorSubsystem.setPower(0.35);
    }
    public void end(boolean interupted){
        m_conveyorSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kRightBumperPort); //pls change// no
    } 
}
