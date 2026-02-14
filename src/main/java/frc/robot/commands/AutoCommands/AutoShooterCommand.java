package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightSubsystem limelight;
    private GenericHID controller;
    public AutoShooterCommand(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelight, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.limelight = limelight;
        this.controller = controller;

        addRequirements(shooterSubsystem);
    }

    @Override 
    public void initialize() {
        
    }

    @Override
    public void execute () {
        double tA = limelight.getTa();
        shooterSubsystem.setPoint(shooterSubsystem.calculateRPM(tA));

        shooterSubsystem.updateError(); 
        shooterSubsystem.setPower(
        shooterSubsystem.getOutput() > 1 ? 1
        : shooterSubsystem.getOutput() < 0 ? 0
        : shooterSubsystem.getOutput()
        );
    }

    @Override
    public void end (boolean interrupted) {
        shooterSubsystem.shutdown();
        shooterSubsystem.resetPID();
    }

    @Override
    public boolean isFinished() {
        return !controller.getRawButton(GamepadConstants.kXButtonPort);
    }
}
