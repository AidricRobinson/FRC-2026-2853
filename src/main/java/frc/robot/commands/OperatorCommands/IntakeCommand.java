package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand  extends Command{
    private IntakeSubsystem intakeSubsystem;
    private GenericHID controller;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, GenericHID m_controller){
        this.intakeSubsystem = intakeSubsystem;
        controller = m_controller;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setPoint(3500);
    }

    @Override
    public void execute(){
        intakeSubsystem.updateError();
        intakeSubsystem.setPower(
            intakeSubsystem.getOutput() > 1 ? 1
            : intakeSubsystem.getOutput() < 0 ? 0
            : intakeSubsystem.getOutput()
        );
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.reset();
        intakeSubsystem.shutdown();
    }

    @Override
    public boolean isFinished(){
        return !controller.getRawButton(YuanConstants.BT_B);
    }
}
