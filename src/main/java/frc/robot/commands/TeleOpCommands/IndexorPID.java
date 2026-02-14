package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.IndexorSubsystem;

public class IndexorPID extends Command {
    private IndexorSubsystem indexorSubsystem;
    private GenericHID controller;
    

    public IndexorPID(IndexorSubsystem indexorSubsystem, GenericHID controller) {
        this.indexorSubsystem = indexorSubsystem;
        this.controller = controller;

        addRequirements(indexorSubsystem);
    }

    @Override
    public void initialize() {
        indexorSubsystem.setPoint(1000);
    }

    @Override
    public void execute() {
        indexorSubsystem.updateError(); 
        indexorSubsystem.setPower(
        indexorSubsystem.getOutput() > 1 ? 1
        : indexorSubsystem.getOutput() < 0 ? 0
        : indexorSubsystem.getOutput()
        );
        
    }
    @Override
    public void end(boolean interrupted) {
        indexorSubsystem.reset();
        indexorSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kLeftBumperPort);
    }



    

    
}
