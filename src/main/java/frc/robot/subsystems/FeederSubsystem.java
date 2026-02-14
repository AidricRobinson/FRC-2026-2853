package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class FeederSubsystem extends SubsystemBase {
    private SparkFlex feeder;
    private SparkFlexConfig feederConfig;
    private double testOutput;
    private PIDController pidController;

    public FeederSubsystem () {
        feeder = new SparkFlex(PortConstants.feederPort, MotorType.kBrushless);
        feederConfig = new SparkFlexConfig();
        feederConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        testOutput = 0;
    }
    public void setPower(double power) {
        feeder.set(power);
    }
    public void setTestOutput() {
        feeder.set(testOutput);
    }
    public void testOutputShutDown() {
        testOutput = 0;
    }
    public void shutdown() {
        feeder.set(0);
    }
    public void increaseSpeed() {
        testOutput += 0.05;
    }
    public void decreaseSpeed() {
        testOutput -= 0.05;
    }
    public double getRPM() {
        return feeder.getEncoder().getVelocity();
    }
    public double getTestOutput() {
        return testOutput;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Test Output", testOutput);
        SmartDashboard.putNumber("Feeder RPM", getRPM());
        SmartDashboard.updateValues();
    }
    public double getError() {
        return pidController.getError();
    }
    public double getOutput() {
        return pidController.calculate(getRPM(), getSetpoint());
    }
    public double getSetpoint() {
        return pidController.getSetpoint();
    }
    public void updateError() {
        pidController.getD();
        pidController.calculate(getRPM(), getSetpoint());
    }
    public void reset() {
        pidController.reset();
    }
}
