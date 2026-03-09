package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SteepShootCommand extends Command{
    private CommandSwerveDrivetrain swerve;
    private ShooterSubsystem shooterSubsystem;
    private IndexorSubsystem indexorSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;
    private Timer timer;

<<<<<<< HEAD
    public SteepShootCommand (ShooterSubsystem shooterSubsystem, IndexorSubsystem indexorSubsystem, HoodSubsystem hoodSubsystem, LimelightSubsystem limelightSubsystem, GenericHID controller) {
=======
    public SteepShootCommand (CommandSwerveDrivetrain swerve, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IndexorSubsystem indexorSubsystem, HoodSubsystem hoodSubsystem, GenericHID controller) {
        this.swerve = swerve;
>>>>>>> 6e41fb68a28ad1b3623bcece7474293deeefa99f
        this.shooterSubsystem = shooterSubsystem;
        this.indexorSubsystem = indexorSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;
        
        addRequirements(shooterSubsystem, indexorSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();
        shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
        indexorSubsystem.setPoint(3000);
        hoodSubsystem.setPoint(AutoConstants.kSteepShootingAngle);
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();
        indexorSubsystem.updateError();
        hoodSubsystem.updateError();

        shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));

        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        hoodSubsystem.setPower(
            hoodSubsystem.getOutput()
        );
        
<<<<<<< HEAD
        if (timer.get() >= 1.5) {
=======
        if (timer.get() >= 2) {
            storageSubsystem.setPower(0.25);
>>>>>>> 6e41fb68a28ad1b3623bcece7474293deeefa99f
            
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
        return !controller.getRawButton(YuanConstants.SideTop);
    }
}