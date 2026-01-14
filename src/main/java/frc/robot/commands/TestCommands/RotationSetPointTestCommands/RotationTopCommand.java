package frc.robot.commands.TestCommands.RotationSetPointTestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotationConstants;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.*;

public class RotationTopCommand extends Command {
    private RotationSubsystem m_RotationSubsystem;

    public RotationTopCommand(RotationSubsystem rotationSubsystem){
        m_RotationSubsystem = rotationSubsystem;

        addRequirements(m_RotationSubsystem);
    }
    @Override
    public void initialize(){
        m_RotationSubsystem.setPoint(RotationConstants.kRotationUpPosition);
    }
    @Override
    public void execute(){
        m_RotationSubsystem.setPower(
            Math.abs(m_RotationSubsystem.getDegreeOutput()) > RotationConstants.kMaximumOutput ? 
            Math.copySign(RotationConstants.kMaximumOutput, m_RotationSubsystem.getDegreeOutput()) 
            : m_RotationSubsystem.getDegreeOutput()); //PLEASE CHECK THIS 
    }
    @Override
    public void end(boolean interupted){
        m_RotationSubsystem.shutdown();
        System.out.println("Rotation finished");
    }
    @Override
    public boolean isFinished(){
        return Math.abs(m_RotationSubsystem.getPIDError()) <= RotationConstants.kRotationTolerance; // CHANGE THIS 

    }
}

