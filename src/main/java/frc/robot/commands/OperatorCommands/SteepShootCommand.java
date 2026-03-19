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
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;
    private GenericHID controller;

    public SteepShootCommand (ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, LimelightSubsystem limelightSubsystem, GenericHID controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.controller = controller;
        
        addRequirements(shooterSubsystem, hoodSubsystem);
    }
    @Override
    public void initialize() {
        // shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
        hoodSubsystem.setPoint(AutoConstants.kHoodHighArcAngle);
    }
    @Override
    public void execute() {
        shooterSubsystem.updateError();
        hoodSubsystem.updateError();

        // shooterSubsystem.setPoint(shooterSubsystem.calculateSteepRPM(swerve.getPoseR()));
        shooterSubsystem.setPoint(1000);

        shooterSubsystem.setPower(
            shooterSubsystem.getOutput() > 1 ? 1
            : shooterSubsystem.getOutput() < 0 ? 0
            : shooterSubsystem.getOutput()
        );

        hoodSubsystem.setPower(
            hoodSubsystem.getDownOutput()
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
        return !controller.getRawButton(YuanConstants.BT_C);
    }
}