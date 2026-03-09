package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class LaunchFuelCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private StorageSubsystem storageSubsystem;
    private IndexorSubsystem indexorSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;
    private Timer timer;

    public LaunchFuelCommand (ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IndexorSubsystem indexorSubsystem, HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;
        
        addRequirements(shooterSubsystem, storageSubsystem, indexorSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();
        shooterSubsystem.setPoint(4500);
        indexorSubsystem.setPoint(3000);
        hoodSubsystem.setPoint(AutoConstants.kLaunchShootingAngle);
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();
        indexorSubsystem.updateError();
        hoodSubsystem.updateError();

        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        hoodSubsystem.setPower(
            hoodSubsystem.getOutput()
        );
        
        if (timer.get() >= 1.5) {
            storageSubsystem.setPower(0.25);
            
            indexorSubsystem.setPower(
                indexorSubsystem.getOutput() > 1 ? 1
                : indexorSubsystem.getOutput() < 0 ? 0
                : indexorSubsystem.getOutput()
            );
        }
    }
    @Override
    public void end (boolean interrupted) {
        shooterSubsystem.shutdown();
        indexorSubsystem.shutdown();
        storageSubsystem.shutdown();
        hoodSubsystem.shutdown();

        shooterSubsystem.resetPID();
        indexorSubsystem.reset();
        storageSubsystem.reset();
        hoodSubsystem.resetPID();
    }
    @Override
    public boolean isFinished() {
        return !controller.getRawButton(YuanConstants.BT_C);
    }
}