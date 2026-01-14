package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.PortConstants;

public class ConveyorSubsystem extends SubsystemBase {
    private TalonFX conveyorMotor;
    private double testSpeed;
    
    public ConveyorSubsystem(){
        conveyorMotor = new TalonFX(PortConstants.conveyorMotorPort);
        conveyorMotor.setNeutralMode(NeutralModeValue.Brake);
        testSpeed= 0.2;
    }
    public void setPower(double power){
        conveyorMotor.set(power);
    }


    public void setMotorTestSpeed() {
        conveyorMotor.set(testSpeed);
    }
    public void setMotorTestSpeedNeg() {
        conveyorMotor.set(-testSpeed);
    }
    public void upSpeed() {
        testSpeed += 0.05;
    }
    public void downSpeed() {
        testSpeed -= 0.05;
    }
    public void testSpeedShutdown() {
        testSpeed = 0;
    }

    public void shutdown(){
        conveyorMotor.set(0);
    }
}