package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignHubCommand extends Command {
    private final CommandSwerveDrivetrain swerve;

    private PIDController pidController;

    // private double offset;
    
    // private double currentAngle;
    // private double translationalSpeed;
    private double output;

    public AlignHubCommand (CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        pidController = new PIDController(0.01, 0.00000001,0);

        output = 0;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(swerve.getHubTurningAngle());

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
