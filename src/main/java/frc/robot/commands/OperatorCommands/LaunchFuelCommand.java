package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchFuelCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;

    public LaunchFuelCommand (ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;
        
        addRequirements(shooterSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        shooterSubsystem.setPoint(3000);
        hoodSubsystem.setPoint(AutoConstants.kHoodLaunchAngle);
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();
        hoodSubsystem.updateError();

        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        hoodSubsystem.setPower(
            hoodSubsystem.getUpOutput()
        );
        
  
    }
    @Override
    public void end (boolean interrupted) {
        shooterSubsystem.shutdown();
        hoodSubsystem.shutdown();

        shooterSubsystem.resetPID();
        hoodSubsystem.resetPID();
    }
    @Override
    public boolean isFinished() {
        return !controller.getRawButton(YuanConstants.BT_A);
    }
}