package frc.robot.commands.TestCommands.FeederTestCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederTestSpeedIncrease extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final FeederSubsystem feederSubsystem;
    private GenericHID controller;

    public FeederTestSpeedIncrease (FeederSubsystem feederSubsystem, GenericHID controller) {
        this.feederSubsystem = feederSubsystem;
        this.controller = controller;
        
        addRequirements(feederSubsystem);
    }
    @Override
    public void initialize() {
        feederSubsystem.increaseSpeed();
    }
    @Override
    public void execute() {

    }
    @Override
    public void end (boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return controller.getPOV() == GamepadConstants.kDpadUp;
    }
}
