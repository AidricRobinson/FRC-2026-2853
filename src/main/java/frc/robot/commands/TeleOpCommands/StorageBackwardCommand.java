package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class StorageBackwardCommand extends Command{
    private StorageSubsystem m_conveyorSubsystem;
    private GenericHID controller;
    public StorageBackwardCommand(StorageSubsystem conveyorSubsystem, GenericHID m_controller){
        m_conveyorSubsystem = conveyorSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        m_conveyorSubsystem.setPower(-0.3);
    }
    public void end(boolean interupted){
        m_conveyorSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);//pls change
    }
}
