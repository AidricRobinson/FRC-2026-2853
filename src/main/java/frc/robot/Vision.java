/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.security.DrbgParameters.Reseed;
import java.util.List;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.DriveFeedforwards;

public class Vision{
    CommandSwerveDrivetrain drivetrain;

    // private final PhotonCamera camRight;
    private final PhotonCamera camLeft;

    // public final PhotonPoseEstimator photonPoseEstimatorRight;
    public final PhotonPoseEstimator photonPoseEstimatorLeft;

    double latestRightX;
    double latestRightY;
    double latestRightT;

    double latestLeftX;
    double latestLeftY;
    double latestLeftT;

    private Matrix<N3, N1> curStdDevs;

    //This is the offset of the camera(s) to the center of the robot
    //
    //
    //
    //      TRANSLATION MEASUREMENTS ARE NOT CORRECT. PLEASE FIX LATER
    //      The rotation measurements should be correct..?
    //
    //
    //
    private static final Transform3d LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.800), Units.inchesToMeters(-8.517), Units.inchesToMeters(43.3)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(20)));

    private static final Transform3d RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.760), Units.inchesToMeters(-12.707), Units.inchesToMeters(43.3)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(20)));

    public Vision(CommandSwerveDrivetrain drivetrain) { // Drive train used to be in parameters
        this.drivetrain =drivetrain ;

        //You set this inside of the Photon Vision localhost site
        camLeft = new PhotonCamera("leftLimelight3");
        // camRight = new PhotonCamera("rightLimelight3G");

        //Make sure this is updated to the current year
        //See docs for how to do this better and set origin for red alliance? I did not do this yet
        var layout =  AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        //Not much to this. Im not sure if I am using the right stratagey for two cameras, as the code I used does not put a valid stratagy
        // photonPoseEstimatorRight = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, LEFT_CAMERA_TO_CENTER);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, LEFT_CAMERA_TO_CENTER);

        //This is just if all else fails, then go single camera mode and use the most clear one (I think thats what it means)
        // photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        boolean LimelightMethodRan = false;

    }
    // public void update()  {
    //     //A bunch of fancy code basically trying to see if the pose estimator has a valid update to make, and updates the swerves pose if it does
    //     // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = photonPoseEstimatorRight.update(camRight.getLatestResult());
    //     // if (optionalEstimatedPoseRight.isPresent()) {
    //     //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
    //     //     drivetrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    //     //     latestRightX = estimatedPose.estimatedPose.getX();
    //     //     latestRightY = estimatedPose.estimatedPose.getY();
    //     //     latestRightT = estimatedPose.estimatedPose.getRotation().getAngle();
    //     // }

    //     final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = photonPoseEstimatorLeft.update(camLeft.getLatestResult());
    //     if (optionalEstimatedPoseLeft.isPresent()) {
    //         final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
    //         drivetrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    //         latestLeftX = estimatedPose.estimatedPose.getX();
    //         latestLeftY = estimatedPose.estimatedPose.getY();
    //         latestLeftT = estimatedPose.estimatedPose.getRotation().getAngle();
    //     }

    // }
  
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Vision.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Vision.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.Vision.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void periodic(){
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camLeft.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorLeft.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonPoseEstimatorLeft.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        
                    });
        }
        
    }
   
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimatorLeft.update(camLeft.getAllUnreadResults().get(0));
    // }

    
}

  