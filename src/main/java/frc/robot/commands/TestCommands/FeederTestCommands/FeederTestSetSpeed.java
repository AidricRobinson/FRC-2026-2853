package frc.robot.commands.TestCommands.FeederTestCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederTestSetSpeed extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final FeederSubsystem feederSubsystem;
    private GenericHID controller;

    public FeederTestSetSpeed (FeederSubsystem feederSubsystem, GenericHID controller) {
        this.feederSubsystem = feederSubsystem;
        this.controller = controller;

        addRequirements(feederSubsystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute () {
        feederSubsystem.setTestOutput();
    }
    @Override
    public void end(boolean interrupted) {
        feederSubsystem.shutdown();
    }
    @Override
    public boolean isFinished() {
        return !(controller.getPOV() == GamepadConstants.kDpadLeft);
    }

}
