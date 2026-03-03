// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.OLDPoseEstimatorSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

import java.security.KeyPair;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;


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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 




                          // TIMED ROBOT STARTS




public class Robot extends TimedRobot { 

  private Vision vision;
  private DriveSubsystem drivetrain;

  public static double lLiftTemp;
  public static double rLiftTemp;

  public static double lift_Position; 

  public static double rotCmmd;
  public static double distanceToTarget;

// - - - - - - - - Basic Robot Declarations - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  //public static Joystick stick1 = new Joystick(0);
  public static Joystick stick2 = new Joystick(1);

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Intake IntakeActive;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - -


public static double test = 0;


// - - - - - - - - - - Intake Timers and Booleans - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  
 

// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public static Timer autoOutTimer = new Timer();
public static boolean autoOutActive = false;

public static Timer autoDriveTimer = new Timer();
//public static boolean autoOutActive = false;





// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public static boolean intakeZero = false;
  public static boolean intakeDown = false;
  public static boolean intakeStop = false;
  public static boolean climbButtonPressed = false;
  public static boolean autonLatch = false;
  public static boolean fullAuton = false;
  public static boolean rotOverRide = false;
  
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


public static double alliTest = 0;


// - - - - - - Camera Declarations - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  // Read in relevant data from the Camera
  public static boolean hasTargets = false;
  double targetYaw = 0.0;
  double targetRange = 0.0;

  public static double Ydrive;
  public static double rotate;

  // Change this to match the name of your camera
  public static PhotonCamera Cam_1 = new PhotonCamera("OV9281");
  public static PhotonCamera Cam_2 = new PhotonCamera("OV9281_2");
  
  public static boolean rotateLatch = false;
  public static boolean  yLatch = false;

  // The field from AprilTagFields will be different depending on the game.
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



// - - - - - - - Sensor Declaration - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


  DigitalInput fIntakeSwitch = new DigitalInput(0);
  
  DigitalInput ampLoadSwitch = new DigitalInput(1);

  public static boolean fIntakeSwitchStat;
    
  public static boolean ampLoadSwitchStat;
  


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

   

                            // ROBOT INI STARTS



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    SmartDashboard.putString("Vision", "None");

    vision = new Vision();
    drivetrain = new DriveSubsystem();

    m_robotContainer = new RobotContainer();

    IntakeActive = new Intake();

    SmartDashboard.putData("Field", DriveSubsystem.m_field);
   
    lift_Position = DriveConstants.liftStart;
   
    //DriveSubsystem.canandgyro.startCalibration();

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    DataLogManager.start();
    DataLogManager.logNetworkTables(true); // Log Netwrok Table Data


    

    
    

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

//Optional <Alliance> ally = DriverStation.getAlliance();   

// - - - - - - TIMER RESETS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  

   
    autoOutTimer.reset();
    autoDriveTimer.reset();
    

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/*if(ally.get() == Alliance.Blue){
  SmartDashboard.putString("Alliance", "Blue");


}else if(ally.get() == Alliance.Red){
  SmartDashboard.putString("Alliance", "Red");


}
*/


// - - - - - SMART DASHBOARD OUTPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    SmartDashboard.putNumber("camYSpeed", Camera.camYSpeed);
    SmartDashboard.putNumber("CamRotSpeed", Camera.camRotSpeed);
    SmartDashboard.putNumber("Turn Ang", DriveSubsystem.canandgyro.getYaw() * (360) );

    SmartDashboard.putNumber("LF Encoder", DriveSubsystem.m_frontLeft.readEncoders());
    SmartDashboard.putNumber("RF Encoder", DriveSubsystem.m_frontRight.readEncoders());
    SmartDashboard.putNumber("RL Encoder", DriveSubsystem.m_rearLeft.readEncoders());
    SmartDashboard.putNumber("RR Encoder", DriveSubsystem.m_rearRight.readEncoders());

    SmartDashboard.putNumber("Intake Lift Position", Intake.intakeLiftPosition());
    SmartDashboard.putNumber("Target Position", Launcher.targetPosition());

    SmartDashboard.putNumber("LeftLaunch Current", Launcher.m_launcherSpark13.getOutputCurrent());
    SmartDashboard.putNumber("RightLaunch Current", Launcher.m_launcherSpark12.getOutputCurrent());

    SmartDashboard.putBoolean("Full Auton", fullAuton);

    SmartDashboard.putNumber("leftY", RobotContainer.m_driverController.getLeftY());
    //SmartDashboard.putNumber("leftYstick1", stick1.getRawAxis(Wire.leftStickY));
    //SmartDashboard.putNumber("leftXstick1", stick1.getRawAxis(Wire.leftStickX));
    //SmartDashboard.putNumber("rightXstick1", stick1.getRawAxis(Wire.rightStickX));
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
      
    //DriveConstants.liveSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -     


  }




                            // ROBOT PERIODIC STARTS




 


  // - - - - - - - - Robot Periodic - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    drivetrain.periodic();

   

// - - - - - - - - -POSE ESTIMATOR - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        // Correct pose estimate with vision measurements
        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
          
                est -> {
                  SmartDashboard.putString("Vision", "Present");
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs();

                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                            Pose2d pose = est.estimatedPose.toPose2d();
                            String table = "Vision Measurment/";
                            SmartDashboard.putNumber(table + "X", pose.getX());
                            SmartDashboard.putNumber(table + "Y", pose.getY());
                            SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
        
                            

                });

      // Correct pose estimate with vision measurements
        var visionEst2 = vision.getEstimatedGlobalPose2();
        visionEst2.ifPresent(
          
                est -> {
                  SmartDashboard.putString("Vision", "Present");
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs();

                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                            Pose2d pose = est.estimatedPose.toPose2d();
                            String table = "Vision Measurment/";
                            SmartDashboard.putNumber(table + "X", pose.getX());
                            SmartDashboard.putNumber(table + "Y", pose.getY());
                            SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
        
                            

                });






        // Test/Example only!
        // Apply an offset to pose estimator to test vision correction
        // You probably don't want this on a real robot, just delete it.
        if (stick2.getRawButton(Wire.startButton)) {
          SmartDashboard.putString("Vision", "Start");
            var disturbance =
                    new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
            drivetrain.resetPose(drivetrain.getPose().plus(disturbance), false);
        }

        // Log values to the dashboard
        drivetrain.log();

      

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



    fIntakeSwitchStat = fIntakeSwitch.get();
    ampLoadSwitchStat = ampLoadSwitch.get();

  



// - - - - - - - SMART DASHBOARD OUTPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    SmartDashboard.putNumber("Turn Ang", DriveSubsystem.canandgyro.getYaw() * (360) );
    SmartDashboard.putNumber("LF Encoder", DriveSubsystem.m_frontLeft.readEncoders());
    SmartDashboard.putNumber("RF Encoder", DriveSubsystem.m_frontRight.readEncoders());
    SmartDashboard.putNumber("RL Encoder", DriveSubsystem.m_rearLeft.readEncoders());
    SmartDashboard.putNumber("RR Encoder", DriveSubsystem.m_rearRight.readEncoders());

    SmartDashboard.putNumber("Intake Lift Position", Intake.intakeLiftPosition());
    SmartDashboard.putNumber("Target Position", Launcher.targetPosition());
   
    SmartDashboard.putNumber("Drive Speed", DriveConstants.kMaxSpeedMetersPerSecond);

    SmartDashboard.putBoolean("intakeSwitch", fIntakeSwitchStat);
    SmartDashboard.putBoolean("intakeSwitch2", ampLoadSwitchStat);

    SmartDashboard.putNumber("ArmLift Current", Intake.m_intakeLiftSpark.getOutputCurrent());

    SmartDashboard.putNumber("LeftLaunch Current", Launcher.m_launcherSpark13.getOutputCurrent());
    SmartDashboard.putNumber("RightLaunch Current", Launcher.m_launcherSpark12.getOutputCurrent());

    //SmartDashboard.putNumber("Lift Current", Intake.m_ClimberSpark.getOutputCurrent());
    //SmartDashboard.putNumber("Climb Pos", Intake.ClimbPosition());

    SmartDashboard.putNumber("leftY", RobotContainer.m_driverController.getLeftY());
    //SmartDashboard.putNumber("leftYstick1", stick1.getRawAxis(Wire.leftStickY));
    //SmartDashboard.putNumber("leftXstick1", stick1.getRawAxis(Wire.leftStickX));
    //SmartDashboard.putNumber("rightXstick1", stick1.getRawAxis(Wire.rightStickX));
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    SmartDashboard.putNumber("Left Motor Speed", Launcher.m_launcherEncoder13.getVelocity());
    SmartDashboard.putNumber("Right Motor Speed", Launcher.m_launcherEncoder12.getVelocity());

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





 /*if(Arm.ArmPosition() > DriveConstants.armLift_1 ){
DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.lowSpeed;
 }else if(stick2.getRawButton(Wire.lBumper)){
DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.madMax;
 }else if(Arm.ClimbPosition() < -5){
  DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.lowSpeed;
 }else{
DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.highSpeed;
 }
 */










  // - - - - - - - - Intake Buttons - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

 
                       // - - - - - - - ZERO INTAKE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - -

 
 
  if(RobotContainer.m_driverController.getRawButton(Wire.aButton)) {
    Intake.IntakeOut();
    
  }





                   // - - - - - - - PUT INTAKE DOWN - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - 
  
  if(RobotContainer.m_driverController.getRawButton(Wire.yButton)) {  
    Intake.IntakeIn();
 }


                  



                  // - - - - - - - INTAKE IN - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - 

 if((RobotContainer.m_driverController.getPOV() == Wire.upDpad)) {
  Intake.intakeUpPos();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



                  // - - - - - - - INTAKE OUT - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - 

 if((RobotContainer.m_driverController.getPOV() == Wire.downDpad)) {
  Intake.intakeDownPos();
}

 


                 // - - - - - - - STOP INTAKE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - 


 if(RobotContainer.m_driverController.getRawButton(Wire.xButton)) {  
  Intake.IntakeStop(); 
 }


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -










//- - - - - - - - Launcher Buttons - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

                    // - - - - - Launch In - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  if((stick2.getPOV() == Wire.downDpad)) {
  Launcher.LauncherIn(); 
 }




                    // - - - - - Launch Out - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


 if((stick2.getPOV() == Wire.upDpad)) {
  Launcher.LauncherOut();
 }


                    // - - - - - Launch Stop - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

 if((stick2.getPOV() == Wire.leftDpad)) {
  Launcher.LauncherStop(); 
 }
             


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






//- - - - - - - - Targeting Buttons - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

                    // - - - - - Target Angle Increase - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


 if(stick2.getRawButton(Wire.aButton)) {
    Launcher.TargetIncrease();
    
  }

                    // - - - - - Target Angle Decrease - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


 if(stick2.getRawButton(Wire.bButton)) {
    Launcher.TargetDecrease();
    
  }

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






// - - - - - -  - - - Automatic Rotation and Drive - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


if(RobotContainer.m_driverController.getRawButton(Wire.bButton)) {

  rotOverRide = true;

  var currentPos = DriveSubsystem.getPose2();
  var currentRot =  DriveSubsystem.canandgyro.getRotation2d();
  var kP = 0.005; //P gain must be tuned
  double rotError;
  
 

    double distanceToTarget = PhotonUtils.getDistanceToPose(currentPos, DriveSubsystem.blueHub);
    Rotation2d targetYaw = PhotonUtils.getYawToPose(currentPos,DriveSubsystem.blueHub);

    rotError = targetYaw.minus(currentRot).getDegrees();
    rotCmmd = rotError * kP ;
     


}else{

rotOverRide = false;

}


// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -







/*  if((!fIntakeSwitchStat && ampLoadSwitchStat)) {
  
  intakeOutActive = true;
}

 if(intakeOutActive) {
  Intake.IntakeOut();
  intakeSwitchTimer.start();
}

 if((intakeOutTimer.get() >= DriveConstants.intakeOutTimeLimit) || (!ampLoadSwitchStat)){
  Intake.IntakeStop();
  intakeOutActive = false;
  intakeSwitchTimer.stop();
  intakeSwitchTimer.reset();
  
  
}
*/








// - - - - - - - Camera Buttons - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  

// 2-10-26 Date changed 

   /*if((stick2.getRawButton(Wire.yButton))) {
   Camera.getRobotPose();
   yLatch = true ;
  }else if (!stick2.getRawButton(Wire.yButton) && (yLatch)) {
    //DriveSubsystem.drive(0, 0, 0, true);
    yLatch = false;
   }



// 2-10-26 Date changed 

  /* 
   if((stick2.getRawButton(Wire.aButton))) {
    Camera.getRobotRot();
    rotateLatch = true ;
   }else if (!stick2.getRawButton(Wire.aButton) && (rotateLatch)) {
     rotateLatch = false;
   }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
*/



// - - - - - - - Climb Buttons - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    /*lLiftTemp = -(stick2.getRawAxis(Wire.rightStickY));
   if(lLiftTemp > 0.5){
     lift_Position = lift_Position + DriveConstants.manualClimbSpeed;
     DriveConstants.liftStart = lift_Position;
     Arm.Climb();
   }
   else if (lLiftTemp < - 0.5){
     lift_Position = lift_Position - (DriveConstants.manualClimbSpeed);
     DriveConstants.liftStart = lift_Position;
     Arm.Climb();
   }
*/

  /*      lLiftTemp = -(stick2.getRawAxis(Wire.rightStickY));
   if(lLiftTemp > 0.5){
     lift_Position = lift_Position + DriveConstants.manualClimbSpeed;
     DriveConstants.liftStart = lift_Position;
     Intake.Climb();
   }
   else if (lLiftTemp < - 0.5){
     lift_Position = lift_Position - (DriveConstants.manualClimbSpeed);
     DriveConstants.liftStart = lift_Position;
     Intake.Climb();
   }

*/
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



// - - - - - - - Climb Auto Down - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  



/*if((stick2.getRawButton(Wire.xButton))) {
  climbButtonPressed = true;
  
  }
*/

 /*  if(climbButtonPressed == true){
    lift_Position = DriveConstants.climbDownLimit;
    DriveConstants.liftStart = lift_Position;
    Intake.Climb();
    climbButtonPressed = false;
   

  }

*/

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


}








// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  @Override
  public void disabledPeriodic() {}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -





// - - - - - - - AUTON MODES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

   

    


  }



  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
    fullAuton = SmartDashboard.getBoolean("Full Auton", fullAuton);
   

    if(fullAuton == true){
     if(!autonLatch){
      autoDriveTimer.start();

     }

     if(autoDriveTimer.get() > DriveConstants.autoDriveTimeLimit){

      autoOutActive = true;
     }

  
    
     if(autoOutActive) {
      Intake.IntakeOut();
      autoOutTimer.start();
    }
    
     if(autoOutTimer.get() >= DriveConstants.autoOutTimeLimit){
      autoOutTimer.stop();
      autoOutTimer.reset();
      autoDriveTimer.stop();
      autoDriveTimer.reset();
      Intake.IntakeStop();
      autoOutActive = false;
      autonLatch = true;
      
    }


  }

  }



  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }


  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
