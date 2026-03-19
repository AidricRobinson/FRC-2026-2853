package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePID extends Command{
    IntakeSubsystem intakeSubsystem; 
    GenericHID controller; 

    public IntakePID (IntakeSubsystem intakeSubsystem, GenericHID controller) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller = controller;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPoint(1000);
    }

    @Override 
    public void execute() {
        intakeSubsystem.updateError();
        intakeSubsystem.setPower(
            intakeSubsystem.getOutput()
        );
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.shutdown();
        intakeSubsystem.reset();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);
    }
    
}
