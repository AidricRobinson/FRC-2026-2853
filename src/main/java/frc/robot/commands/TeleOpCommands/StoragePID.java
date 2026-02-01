package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.StorageSubsystem;

public class StoragePID extends Command {
    private StorageSubsystem storageSubsystem;
    private GenericHID controller;
    

    public StoragePID(StorageSubsystem storageSubsystem, GenericHID controller) {
        this.storageSubsystem = storageSubsystem;
        this.controller = controller;

        addRequirements(storageSubsystem);
    }

    @Override
    public void initialize() {
        storageSubsystem.setPoint(2500);
    }

    @Override
    public void execute() {
        storageSubsystem.updateError(); 
        storageSubsystem.setPower(
        storageSubsystem.getOutput() > 1 ? 1
        : storageSubsystem.getOutput() < 0 ? 0
        : storageSubsystem.getOutput()
        );
        
    }
    @Override
    public void end(boolean interrupted) {
        storageSubsystem.reset();
        storageSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kRightBumperPort);
    }



    

    
}
