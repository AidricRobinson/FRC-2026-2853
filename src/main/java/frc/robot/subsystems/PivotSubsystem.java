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

public class PivotSubsystem extends SubsystemBase {
    private SparkFlex pivot;
    private SparkFlexConfig pivotConfig;
    private PIDController pidController;
    private double testOutput;

    public PivotSubsystem () {
        pivot = new SparkFlex(PortConstants.pivotPort, MotorType.kBrushless);
        pidController = new PIDController(0, 0, 0);
        testOutput = 0;
        pivotConfig = new SparkFlexConfig();
        pivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getRotation() {
        return pivot.getEncoder().getPosition();
    }
    public void setPower(double power) {
        pivot.set(power);
    }
    public void shutdown() {
        pivot.set(0);
    }

    public void setTestOutput() {
        pivot.set(testOutput);
    }
    public void increaseOutput() {
        testOutput += 0.05;
    }
    public void decreaseOutput() {
        testOutput -= 0.05;
    }
    public void testOutputShutdown () {
        testOutput = 0;
    }
    public double getTestOutput () {
        return testOutput;
    }

    public double getError() {
        return pidController.getError();
    }
    public double getOutput() {
        return pidController.calculate(getRotation(), getSetPoint());
    }
    public double getSetPoint() {
        return pidController.getSetpoint();
    }
    public void setPoint(double target) {
        pidController.setSetpoint(target);
    }
    public void updateError() {
        pidController.getD();
        pidController.calculate(getRotation(), getSetPoint());
    }
    public void resetPID() {
        pidController.reset();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Test Output", testOutput);
        SmartDashboard.putNumber("Pivot Rotations", getRotation());
        SmartDashboard.putNumber("Pivot SetPoint", getSetPoint());
        SmartDashboard.putNumber("Pivot Error", getError());
        SmartDashboard.updateValues();
    }
    
}
