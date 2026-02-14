package frc.robot.commands.TestCommands.FeederTestCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederTestSpeedDecrease extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final FeederSubsystem feederSubsystem;
    private GenericHID controller;

    public FeederTestSpeedDecrease (FeederSubsystem feederSubsystem, GenericHID controller) {
        this.controller = controller;
        this.feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.decreaseSpeed();
    }
    @Override
    public void execute() {

    }
    public void end(boolean interrupted) {

    }
    public boolean isFinished() {
        return controller.getPOV() == GamepadConstants.kDpadDown;
    }
}
