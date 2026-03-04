package frc.robot.commands.TestCommands.HoodTestCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodTestDownCommand extends Command {
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;
    

    public HoodTestDownCommand(HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        hoodSubsystem.setPoint(0);
    }

    @Override
    public void execute() {
        hoodSubsystem.updateError(); 
        hoodSubsystem.setPower(
        hoodSubsystem.getOutput() > 1 ? 1
        : hoodSubsystem.getOutput() < 0 ? 0
        : hoodSubsystem.getOutput()
        );
        
    }
    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.resetPID();
        hoodSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kRightBumperPort);
        
    }



    

    
}
