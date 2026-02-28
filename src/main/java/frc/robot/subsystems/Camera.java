package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.timesync.TimeSyncSingleton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;

public class Camera {

// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  // Read in relevant data from the Camera
  public static boolean hasTargets = false;
  static double targetYaw = 0.0;
  static double targetRange = 0.0;
  public static double camXSpeed = 0.0;
  public static double camYSpeed = 0.0;
  public static double camRotSpeed = 0.0;
  static double cam_OneHeight = 0.3;
  static double cam_OnePitch = 0.0; // radians
  static double cam_OneTargHeight = 0.174625; // meters
  static double cam_OneGoalRange = -0.0444; // meters
  static double rightOffset = 0.0;
  static double leftOffset = 0.0;
  

  static double Y_P = 1.4;
  static double Y_D = 0.05;

  static double angular_P = 0.015;
  static double angular_D = 0;

  static PIDController turnController = new PIDController(angular_P, 0, angular_D);

  static PIDController xDriveController = new PIDController(Y_P, 0, Y_D);

  static PIDController yDriveController = new PIDController(Y_P, 0, Y_D);

  // Change this to match the name of your camera
  public static PhotonCamera Cam_1 = new PhotonCamera("OV9281");
  

  // The field from AprilTagFields will be different depending on the game.
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  public static final Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,180)); 





    

public static void getRobotPose(){

    // Query the latest result from PhotonVision
    var result = Cam_1.getLatestResult();
    hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("has targetCAM", hasTargets);

    if (hasTargets) {

        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();     
        double finalY = bestCameraToTarget.getY();
        camYSpeed = yDriveController.calculate(finalY, cam_OneGoalRange);

        SmartDashboard.putNumber("camYSpeed", Camera.camYSpeed);
        SmartDashboard.putNumber("CamRange", finalY);


    //DriveSubsystem.drive(0, -camYSpeed, 0, true);
    Robot.Ydrive = -camYSpeed;
    }
}





public static void getRobotRot(){

  // Query the latest result from PhotonVision
  var result = Cam_1.getLatestResult();
  hasTargets = result.hasTargets();
  SmartDashboard.putBoolean("has targetCAM", hasTargets);
 
  if (hasTargets) {
  
      camRotSpeed = (turnController.calculate(result.getBestTarget().getYaw()));
      var ang = result.getBestTarget().getYaw();

      SmartDashboard.putNumber("targetAng", ang);
      SmartDashboard.putNumber("CamRotSpeed", Camera.camRotSpeed);
  
      Robot.rotate = -camRotSpeed;


  }
}







/* 
//if (hasTargets) {

    var result = camera_1.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    double targetX = target.getPitch(); // Horizontal angle in degrees (left-right)
    double targetY = target.getYaw();   // Vertical angle in degrees (up-down)

    yaw = target.getYaw();
   // X = target.getX();
    //Y = target.getY();

    }
*/



} 




 
