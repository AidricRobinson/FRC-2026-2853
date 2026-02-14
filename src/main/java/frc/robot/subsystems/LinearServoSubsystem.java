package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearServoSubsystem extends SubsystemBase {
    private Servo servo;

    public LinearServoSubsystem () {
        // eh
    }

    public void setSpeed(double output) {
        servo.setSpeed(output);
    }



}
