package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class IntakePID extends Command {
    private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;
    

    public IntakePID(IntakeSubsystem intakeSubsystem, GenericHID controller) {
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
        intakeSubsystem.getOutput() > 1 ? 1
        : intakeSubsystem.getOutput() < 0 ? 0
        : intakeSubsystem.getOutput()
        );
        
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.reset();
        intakeSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);
    }



    

    
}
