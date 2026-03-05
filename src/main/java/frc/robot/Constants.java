// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;




public final class Constants {



// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  public static class Vision {
    public static final String kCameraName = "OV9281";
    public static final String kCameraName_2 = "OV9281_2";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}

// - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






  public static final class DriveConstants {

     // - - - - - - - - - - - - - - - - Drive Speeds - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
/* 
 public static  double highSpeed = 4; //2.5
 public static  double lowSpeed = .6;
 public static  double liveSpeed =2.3;
 public static  double madMax = 4;
*/
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond= 4.8;// = liveSpeed;//Was 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second






    //- - - - - - - - - - - - Chassis configuration - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public static final double kTrackWidth = Units.inchesToMeters(25);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0; 
    public static final double kBackLeftChassisAngularOffset = 0; 
    public static final double kBackRightChassisAngularOffset = 0; 

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





    // - - - - - - - - - - - - - - - - - SPARK MAX CAN IDs - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 11;


    // - - - Arm Can IDs - - - - - - - - - - - - 
    public static final int kIntakeLiftCanId = 14;
    public static final int kExtendArmCanId = 7;
    public static final int kTargetingCanId = 15;
    public static final int kIntakeCanId = 9;
    public static final int kLauncherCan12Id = 12;
    public static final int kLauncherCan13Id = 13;

    
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





// - - - - - - - - - - - - - - - - Targeting Positions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public static final double tarIncreaseAng_Pos = -0.3;  
    public static final double tarDecreaseAng_Pos = 0;         
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 






    // - - - - - - - - - - - - - - - - Lift Positions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public static final double intakeDown_Pos = 0.2;  // Puts intake down  //-24
    public static final double intakeUp_Pos = 0;         //0 
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


   


 // - - - - - - - - - - - - - - - - Climb Position - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    public static double liftStart = 0;
    public static double climbDownLimit = -14;
 // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -




    // - - - - - - - - - - - - - - - - Robot Drive - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    public static final double speedLimit = 0.1;
    public static final double rotationSpeed = 1;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  




   // - - - - - - - - - - - - - - - - Speeds - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    public static final double intakeInSpeed = -0.45;
    public static final double intakeOutSpeed = 0.45;

    public static final double launcherInSpeed = 50;
    public static final double launcherOutSpeed = 1300;

    public static final double manualClimbSpeed = 1;


   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
   
   


    // - - - - - - - - - - - - - - - - Timers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    public static final double intakeInTimeLimit = 5;
    public static final double intakeOutTimeLimit = 0.2;
    public static final double intakeSwitchTimeLimit = 0.4;
    public static final double zeroExtendTimeLimit = 5;

    public static final double autoOutTimeLimit = 2;
    public static final double autoDriveTimeLimit = 6;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    // - - - - - - - - - - - - - - - - Trajectory - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






    // - - - - - - - - - - - - - - - - Reverse Gyro - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    public static final boolean kGyroReversed = false;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    
  }









  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.098;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 6.75; //(45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //Math.PI
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //Math.PI

    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d (0.311 , 0.311);  ;//(0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d (0.311 , -0.311);   //(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d (-0.311 ,0.311);   // (-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.311, -0.311);  //(-0.546 / 2.0, -0.546 / 2.0);

    public static final double maxModuleSpeed = 4.5; // M/S

    public static final PIDConstants translationConstants = new PIDConstants(8.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(8.0, 0.0, 0.0);
  }

}
