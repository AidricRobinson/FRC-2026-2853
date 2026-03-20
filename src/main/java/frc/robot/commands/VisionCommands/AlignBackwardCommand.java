package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignBackwardCommand extends Command {
    private final CommandSwerveDrivetrain swerve;

    private PIDController pidController;

    // private double offset;
    
    // private double currentAngle;
    // private double translationalSpeed;
    private double output;

    public AlignBackwardCommand (CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        pidController = new PIDController(4, 5,0.1);

        output = 0;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(-180);

    }

    @Override
    public void execute() {
        output = pidController.calculate(swerve.getCurrentAngle());
        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, output, Rotation2d.fromDegrees(swerve.getCurrentAngle()));
        swerve.setRotationalSpeed(output, targetSpeed);
        // translationalSpeed = pidController.calculate(currentAngle, offset);

        // ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, translationalSpeed, Rotation2d.fromDegrees(currentAngle));
        // swerve.driveRobotRelative(targetSpeed);
        System.out.println("ALIGNMENT RUNNING");
        
    }
    @Override
    public void end (boolean isFinished) {
        for (int i = 0; i < 10; i++) {
            System.out.println("ALIGNMENT COMMAND FINISHED");
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pidController.getError()) < 5;
    }
}
