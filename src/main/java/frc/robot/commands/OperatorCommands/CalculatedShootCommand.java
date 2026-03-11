package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CalculatedShootCommand extends Command{
    private CommandSwerveDrivetrain swerve;
    private ShooterSubsystem shooterSubsystem;
    private IndexorSubsystem indexorSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;
    private Timer timer;
    private boolean close;
    private boolean far;

    public CalculatedShootCommand (CommandSwerveDrivetrain swerve, ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.swerve = swerve;
        this.shooterSubsystem = shooterSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;
        far = false;
        close = false;
        
        addRequirements(shooterSubsystem, indexorSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();
        if (swerve.getPoseR() >= 1.75) {
            shooterSubsystem.setPoint(shooterSubsystem.calculateDistanceRPM(swerve.getPoseR()));
            hoodSubsystem.setPoint(AutoConstants.kNormalShootingAngle);
            far = true;
        }
        else if (swerve.getPoseR() < 1.75) {
            shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
            hoodSubsystem.setPoint(AutoConstants.kSteepShootingAngle);
            close = true;
        }
        indexorSubsystem.setPoint(3000);
        
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();
        indexorSubsystem.updateError();
        hoodSubsystem.updateError();

        if (close) {
            shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
        }
        else if (far) {
            shooterSubsystem.setPoint(shooterSubsystem.calculateDistanceRPM(swerve.getPoseR()));

        }
       
        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        hoodSubsystem.setPower(
            hoodSubsystem.getOutput()
        );
        
        if (timer.get() >= 2) {
            
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
        hoodSubsystem.shutdown();

        shooterSubsystem.resetPID();
        indexorSubsystem.reset();
        hoodSubsystem.resetPID();
    }
    @Override
    public boolean isFinished() {
        return !controller.getRawButton(YuanConstants.SideBottom);
    }
}