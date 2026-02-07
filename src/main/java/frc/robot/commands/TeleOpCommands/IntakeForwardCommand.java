package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForwardCommand extends Command{
   private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;
    public IntakeForwardCommand(IntakeSubsystem intakeSubsystem, GenericHID m_controller){
        this.intakeSubsystem = intakeSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        intakeSubsystem.setPower(0.5);
    }
    public void end(boolean interupted){
        intakeSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kRightBumperPort); //pls change// no
    } 
}
