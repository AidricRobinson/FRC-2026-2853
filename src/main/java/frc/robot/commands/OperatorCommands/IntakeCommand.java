package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeCommand  extends Command{
    private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;
    private PivotSubsystem pivotSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, GenericHID m_controller){
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        controller = m_controller;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setPoint(2750);
    }

    @Override
    public void execute(){
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
    public void end(boolean interrupted){
        intakeSubsystem.reset();
        pivotSubsystem.shutdown();
        intakeSubsystem.shutdown();
    }

    @Override
    public boolean isFinished(){
        return !controller.getRawButton(YuanConstants.BT_B);
    }
}
