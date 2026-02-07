package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GamepadConstants;

public class IntakeBackwardCommand extends Command{
    private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;
    public IntakeBackwardCommand(IntakeSubsystem intakeSubsystem, GenericHID m_controller){
        this.intakeSubsystem = intakeSubsystem;
        controller = m_controller;
    }
    public void initialize(){

    }
    public void execute(){
        intakeSubsystem.setPower(-0.5);
    }
    public void end(boolean interupted){
        intakeSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);//pls change
    }
}
