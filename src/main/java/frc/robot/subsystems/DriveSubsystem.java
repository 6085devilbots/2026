// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Array;
import java.util.Arrays;

import com.reduxrobotics.sensors.canandgyro.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Wire;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.*;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// Add other necessary imports as needed

public class DriveSubsystem extends SubsystemBase {

  //private final SimSwerveModule[] modules;

  
  // Create MAXSwerveModules
  public static MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  public static MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  public static MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public static MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  public static SwerveDrivePoseEstimator poseEstimator;

  public static Field2d m_field = new Field2d();
      

  // - - - - - Gyro Sensor - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU(); 
  public static Canandgyro canandgyro = new Canandgyro(18); // Gyro ID

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  // - - - - - - Odometry class for tracking robot pose - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

 public SwerveModulePosition[] robopos() {

  return
  new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()
    
};

 };




// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        getHeading3(),
                        robopos(),
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // - - - - - - PATHPLANNER - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

   // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
   // RobotConfig config;


    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose3, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", m_field);












    /*try{
      config = RobotConfig.fromGUISettings();
    // Configure AutoBuilder last
    AutoBuilder.configure(
            getPose(), // Robot pose supplier
            resetPose(getPose(),false), // Method to reset odometry (will be called if your auto has a starting pose)
            getSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
  } catch (Exception e) {
    // Handle exception as needed
    e.printStackTrace();
  }

  // Set up custom logging to add the current path to a field 2d widget
  PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

  SmartDashboard.putData("Field", m_field);  
*/
  } 

  

  @Override
  public void periodic() { 
    
    // Update the pose estimator in the periodic block
    poseEstimator.update(
      canandgyro.getRotation2d(),  //(360)
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
   
   
    //m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
  }
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





  // - - - - - - Odometry and Current Robot Position - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static Pose2d getPose2() {
    return poseEstimator.getEstimatedPosition();
  }




// - - - - - - - Creates Object Blue Hub  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public static Pose2d blueHub = new Pose2d(
    4.035, // X position in meters  
    4.623,  // Y position in meters   15.167ft
    Rotation2d.fromDegrees(45.0) // Angle in degrees
);

// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





  public SwerveModuleState[] roboState() {

    return
    new SwerveModuleState[] {
    m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()
      
  };
}




  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(roboState());
  }


  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
  

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    //public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
       
    //    String table = "Add Vision/";
    //    SmartDashboard.putNumber(table + "X", visionMeasurement.getX());
    //    SmartDashboard.putNumber(table + "Y", visionMeasurement.getY());
    //    SmartDashboard.putNumber(table + "Heading", visionMeasurement.getRotation().getDegrees());

    //    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    //}

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
        Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {

        String table = "Add Vision/";
        SmartDashboard.putNumber(table + "X", visionMeasurement.getX());
        SmartDashboard.putNumber(table + "Y", visionMeasurement.getY());
        SmartDashboard.putNumber(table + "Heading", visionMeasurement.getRotation().getDegrees());

        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    /**
     * Reset the estimated pose of the swerve drive on the field.
     *
     * @param pose New robot pose.
     * @param resetSimPose If the simulated robot pose should also be reset. This effectively
     *     teleports the robot and should only be used during the setup of the simulation world.
     */



  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPoseEst(Pose2d pose) {
    poseEstimator.resetPosition(
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        canandgyro.getRotation2d(), //(360)   
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -







  // - - - - - - - Drive and Joytstick Code- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public static void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain0

    double rot2;     // Creates second rotation variable to lock out drive code from resetting rot and not using rotCmmd

    if(Robot.rotOverRide){
      rot2 = Robot.rotCmmd;  // If rot overide is true then rotation is told to be our rotation needed to point at the hub

    }else{
      rot2 = rot;  // Says if rotation Override isn't true then rot2 should be the same as the roation variable originally was so robot drive as usual 

    }

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond; // changed from negative to positive
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond; // changed from negative to positive
    double rotDelivered = rot2 * DriveConstants.kMaxAngularSpeed;               // changed from negative to positive

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                canandgyro.getRotation2d())//(canandgyro.getYaw()  ))  //(360)
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -







  // - - - - - - - No Drift Mode - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 






  // - - - - - - Swerve Module States - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -




  // - - - - Resets Drive Encoders To Read The Postion Of Zero - - - - - - - - - - - - - - - - - - - - - - - - 
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






    // - - - - - Reads Drive Encoders - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public Double readFLEncoders() {
      return m_frontLeft.readEncoders();
    }

    public Double readFREncoders() {
      return m_frontRight.readEncoders();
    }

    public Double readRLEncoders() {
      return m_rearLeft.readEncoders();
    }

    public Double readRREncoders() {
      return m_rearRight.readEncoders();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 






  // - - - - - Zeros The Heading Of The Robot - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  public void zeroHeading() {
    canandgyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(canandgyro.getYaw()  ).getDegrees();  //(360)
  }


  public Rotation2d getHeading2() {
    return Rotation2d.fromDegrees(canandgyro.getYaw());
  }

  public Rotation2d getHeading3() {
    return canandgyro.getRotation2d();
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


  public void resetPose3(Pose2d pose) {
    System.out.println(pose);
    poseEstimator.resetPosition(canandgyro.getRotation2d(), robopos(), pose);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return canandgyro.getAngularVelocityYaw() * (DriveConstants.kGyroReversed ? 1.0 : -1.0);
  }
  





  public void resetPose(Pose2d pose, boolean resetSimPose) {
    poseEstimator.resetPosition(getHeading3(),robopos(), pose);
}





 /** Log various drivetrain values to the dashboard. */
    public void log() {
        String table = "Drive/";
        Pose2d pose = getPose();
        SmartDashboard.putNumber(table + "X", pose.getX());
        SmartDashboard.putNumber(table + "Y", pose.getY());
        SmartDashboard.putNumber(table + "Pose Heading", pose.getRotation().getDegrees());

        Rotation2d headng = getHeading3();
        SmartDashboard.putNumber(table + "Gyro Heading", headng.getDegrees());

    }


    

}
