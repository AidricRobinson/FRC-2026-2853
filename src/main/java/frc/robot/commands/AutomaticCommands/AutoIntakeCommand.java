package frc.robot.commands.AutomaticCommands;
// package frc.robot.commands.AutonomousCommands;

// import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoIntakeCommand extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final IntakeSubsystem intakeSubsystem;

//   public AutoIntakeCommand(IntakeSubsystem intakeSubsystem){
//     this.intakeSubsystem = intakeSubsystem;

//     addRequirements(intakeSubsystem);
//   }


//   @Override
//   public void initialize() {
//     intakeSubsystem.setPoint(3000); // placeholder

//   }
//   @Override
//   public void execute() {
//     intakeSubsystem.setPower(
//         intakeSubsystem.getOutput()
//     );
//   }
//   @Override
//   public void end(boolean interrupted) {
//     intakeSubsystem.shutdown();
//   }
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
