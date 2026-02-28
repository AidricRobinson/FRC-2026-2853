package frc.robot.subsystems;


import java.nio.channels.Pipe;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable networkTable;
    private NetworkTable networkTable2;

    public enum Pipeline {
        DRIVER_VIEW(0), APRILTAG(1);
        private int PipelineID;

        private Pipeline(int PipelineID) {
            this.PipelineID = PipelineID;
        }
    }

    public LimelightSubsystem () {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        networkTable2 = NetworkTableInstance.getDefault().getTable("limelight")      ;
        setPipeline(Pipeline.DRIVER_VIEW);
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
    }

    public void setPipeline(Pipeline pipeline) {
        networkTable.getEntry("pipeline").setValue(pipeline.PipelineID);
        networkTable2.getEntry("pipeline").setValue(pipeline.PipelineID);
    }

    // public Pose2d getbotPose2d(){
    //     return networkTable.getEntry("botpose");
    // }
    
    public double getTx() {
        return networkTable.getEntry("tx").getDouble(0.0); // will be changed for kitbot limelight placement
    }

    public double getTx2(){
        return networkTable2.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return networkTable.getEntry("ty").getDouble(0.0);
    }

    public double getTy2() {
        return networkTable2.getEntry("ty").getDouble(0.0);
    }

    public double getTa (){
        return networkTable.getEntry("ta").getDouble(0.0);
    } 

    public double getTa2(){
        return networkTable2.getEntry("ta").getDouble(0.0);
    } 


    @Override
    public void periodic(){
        getTa();
        getTx();
        getTy();
        getTx2();
        getTa2();
        getTy2();
        SmartDashboard.putNumber("tx", getTx());
        SmartDashboard.putNumber("ty", getTy());
        SmartDashboard.putNumber("ta", getTa());
        SmartDashboard.putNumber("tx2", getTx2());
        SmartDashboard.putNumber("ta2", getTa2());
        SmartDashboard.putNumber("ty", getTy2());
        SmartDashboard.updateValues();
    }
}
