package frc.robot.commands.TestCommands.ShooterTestCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PID;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPID extends Command {
    private ShooterSubsystem shooterSubsystem;
    private GenericHID controller;

    public ShooterPID(ShooterSubsystem shooterSubsystem, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setPoint(2500);
        shooterSubsystem.resetPID();
    }

    @Override
    public void execute() {
        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? -1
            : shooterSubsystem.getOutput() < -1 ? -1
            : shooterSubsystem.getOutput()
        );
    }
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kAButtonPort);
    }



    

    
}
