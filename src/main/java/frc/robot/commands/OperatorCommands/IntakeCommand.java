package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeCommand  extends Command{
    private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;
    private PivotSubsystem pivotSubsystem;
    private IndexorSubsystem indexorSubsystem;

    public IntakeCommand(IndexorSubsystem indexorSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, GenericHID m_controller){
        this.indexorSubsystem = indexorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        controller = m_controller;

        addRequirements(intakeSubsystem, indexorSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setPoint(4500);
    }

    @Override
    public void execute(){
        intakeSubsystem.updateError();
        intakeSubsystem.setPower(
            intakeSubsystem.getOutput() > 1 ? 1
            : intakeSubsystem.getOutput() < 0 ? 0.25
            : intakeSubsystem.getOutput()
        );
        // intakeSubsystem.setPower(1);
        if (pivotSubsystem.getPivotEncoder() >= AutoConstants.kPivotDownPosition) {
            pivotSubsystem.setPower(0);
        }
        else {
            pivotSubsystem.setPower(-0.05);
        }

        // indexorSubsystem.setPower(-0.1);
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.reset();
        pivotSubsystem.shutdown();
        intakeSubsystem.shutdown();
        indexorSubsystem.shutdown();
    }

    @Override
    public boolean isFinished(){
        return !controller.getRawButton(YuanConstants.BT_B);
    }
}
