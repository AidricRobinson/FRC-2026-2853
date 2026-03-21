package frc.robot.commands.AutomaticCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private double duration;
    private final Timer timer = new Timer();
    public AutoIntakeCommand (IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, double duration) {
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.duration = duration;

        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();

        intakeSubsystem.setPoint(4000);
    }
    @Override
    public void execute() {
        intakeSubsystem.setPower(duration);

        intakeSubsystem.updateError();
        intakeSubsystem.setPower(
            intakeSubsystem.getOutput() > 1 ? 1
            : intakeSubsystem.getOutput() < 0 ? 0
            : intakeSubsystem.getOutput()
        );
        if (pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition) {
            pivotSubsystem.setPower(0);
        }
        else {
            pivotSubsystem.setPower(-0.05);
        }
    }

    @Override
    public void end (boolean interrupted) {
        intakeSubsystem.shutdown();
        intakeSubsystem.reset();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished () {
        return duration <= timer.get();
    }


}
