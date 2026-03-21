package frc.robot.commands.AutomaticCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotDownCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public AutoPivotDownCommand(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(pivotSubsystem, intakeSubsystem);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotSubsystem.setPower(-0.2);
        intakeSubsystem.setPower(-0.2);
    }
    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.shutdown();
        intakeSubsystem.shutdown();
    }
    @Override
    public boolean isFinished() {
        return pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition;
    }
}
