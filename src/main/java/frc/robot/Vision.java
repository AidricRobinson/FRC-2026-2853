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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision {
    CommandSwerveDrivetrain drivetrain;

    private final PhotonCamera camRight;
    private final PhotonCamera camLeft;

    public final PhotonPoseEstimator photonPoseEstimatorRight;
    public final PhotonPoseEstimator photonPoseEstimatorLeft;

    //This is the offset of the camera(s) to the center of the robot
    //
    //
    //
    //      MEASUREMENTS ARE NOT CORRECT. PLEASE FIX LATER
    //
    //
    //
    private static final Transform3d LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.800), Units.inchesToMeters(-8.517), Units.inchesToMeters(43.3)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(35.895 / 2)));

    private static final Transform3d RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.760), Units.inchesToMeters(-12.707), Units.inchesToMeters(43.3)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-35.895 / 2)));

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        //You set this inside of the Photon Vision localhost site
        camLeft = new PhotonCamera("leftLimelight3G");
        camRight = new PhotonCamera("rightLimelight3");

        //Make sure this is updated to the current year
        //See docs for how to do this better and set origin for red alliance? I did not do this yet
        var layout =  AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        //Not much to this. Im not sure if I am using the right stratagey for two cameras, as the code I used does not put a valid stratagy
        photonPoseEstimatorRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RIGHT_CAMERA_TO_CENTER);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LEFT_CAMERA_TO_CENTER);

        //This is just if all else fails, then go single camera mode and use the most clear one (I think thats what it means)
        photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }
    public void update()  {
        //A bunch of fancy code basically trying to see if the pose estimator has a valid update to make, and updates the swerves pose if it does
        final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = photonPoseEstimatorRight.update(camRight.getLatestResult());
        if (optionalEstimatedPoseRight.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
            drivetrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }

        final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = photonPoseEstimatorLeft.update(camLeft.getLatestResult());
        if (optionalEstimatedPoseLeft.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
            drivetrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }

    }
    public void periodic() {
       
    }
}

  