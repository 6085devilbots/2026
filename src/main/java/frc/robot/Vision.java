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

 import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision {
    public static PhotonCamera Cam_1;
    public static PhotonCamera Cam_2;
    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator photonEstimator2;
    private Matrix<N3, N1> curStdDevs;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
 
     public Vision() {
         Cam_1 = new PhotonCamera(kCameraName);
         Cam_2 = new PhotonCamera(kCameraName_2);

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        Transform3d robotToCam = new Transform3d(new Translation3d(0.07, 0.13, 0.35), new Rotation3d(0,0,0));  // x = 3in  
        //Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,180));
        Transform3d robotToCam2 = new Transform3d(new Translation3d(0.14, 0.13, 0.35), new Rotation3d(0,0,180));

        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
        photonEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2);
         photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         

         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(Cam_1, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }


         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("second");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(Cam_2, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }
     



     }

    
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */

/* 
      public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonEstimator.update();
        
      } */





     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : Cam_1.getAllUnreadResults()) {   // for (var change : camera.getAllUnreadResults()) {
            SmartDashboard.putString("Vision", "gEgp");
             visionEst = photonEstimator.update(change);
             updateEstimationStdDevs(visionEst, change.getTargets()); 

         }

        
         return visionEst;
     }


      public Optional<EstimatedRobotPose> getEstimatedGlobalPose2() {
         Optional<EstimatedRobotPose> visionEst2 = Optional.empty();
         for (var change : Cam_2.getAllUnreadResults()) {   // for (var change : camera.getAllUnreadResults()) {
            SmartDashboard.putString("Vision", "gEgp");
             visionEst2 = photonEstimator2.update(change);
             updateEstimationStdDevs(visionEst2, change.getTargets());
 
           


         }
         return visionEst2;
     }

   
       public class ProjectileTrajectory {
       public static void main(String[] args) {
        double v0 = 7.4676; // Initial velocity (m/s)
        double angleDeg; // Launch angle (degrees)
        double g = 9.81; // Gravity (m/s^2)
        double distance = Robot.distanceToTarget; //x
        double roboHeight = 0.399415;   //y = 15.725in
        double hubHeight = 1.8288; //72 in

        // Calculate y for various x positions
        Math.atan();
         
       }



     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                 curStdDevs = kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }

     
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }


    
    
 }