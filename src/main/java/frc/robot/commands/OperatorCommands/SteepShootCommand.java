package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SteepShootCommand extends Command{
    private CommandSwerveDrivetrain swerve;
    private LimelightSubsystem limelightSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;

    public SteepShootCommand (ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, LimelightSubsystem limelightSubsystem, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.controller = controller;
        
        addRequirements(shooterSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        // shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();

        shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(limelightSubsystem.getTa()));

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
        return !controller.getRawButton(YuanConstants.BT_A);
    }
}