package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Vision;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Intake;
import frc.robot.Vision;
import frc.robot.subsystems.AutoLaunchCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
*/
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoLaunch extends SubsystemBase {

  public static Command Launch;
  public static Command StopLaunch;

  //public static void Launch(Pose2d tarGoal){
  public void Launch(Pose2d tarGoal){

 // return Commands.run(() -> {

  var currentPos = DriveSubsystem.getPose2();
  var currentYaw = DriveSubsystem.canandgyro.getYaw();
  var currentRot =  DriveSubsystem.canandgyro.getRotation2d(); 
  double kP = 0.07615; //P gain must be tuned
  double rotError;
  //double rotError2;
 

  Launcher.m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);
  Launcher.m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);


  PhotonPipelineResult result1 = Vision.Cam_1.getLatestResult();
  PhotonPipelineResult result2 = Vision.Cam_2.getLatestResult();

  if(result1.hasTargets() || result2.hasTargets()){

  Launcher.rotOverRide = true;

 
  double distanceToTarget = PhotonUtils.getDistanceToPose(currentPos, tarGoal);
  Rotation2d targetYaw = PhotonUtils.getYawToPose(currentPos,tarGoal);

  double iniLaunchVel = ProjectileTrajectory.calcInitialVelocity(ProjectileTrajectory.avgLaunchVelocity());
   
  double launchAngle = ProjectileTrajectory.calcLaunchAngle(iniLaunchVel, distanceToTarget, 0.395, 1.524); //DriveConstants.initialHeight
    
  //Launcher.m_targetClosedLoopController.setSetpoint(80, SparkMax.ControlType.kPosition);
  Launcher.m_targetClosedLoopController.setSetpoint(launchAngle, SparkMax.ControlType.kPosition);
    
  double actLaunchAng = Launcher.targetPosition();

  rotError = targetYaw.getDegrees();


  if (rotError > DriveConstants.rotError2) {
    rotError = DriveConstants.rotError2;
  } 

     


  Launcher.rotCmmd = rotError * kP ;

  SmartDashboard.putNumber("Initial Launch Velocity", iniLaunchVel);
  SmartDashboard.putNumber("Launch Ang Encoder", actLaunchAng);
  SmartDashboard.putNumber("Cal Launch Angle", launchAngle);
  SmartDashboard.putNumber("TargetYaw", targetYaw.getDegrees());
  SmartDashboard.putNumber("RotCmmd", Launcher.rotCmmd);
  SmartDashboard.putNumber("rotError", rotError);
  SmartDashboard.putNumber("Distance To Target", distanceToTarget);
 

  
  
  //SmartDashboard.putNumber("Distance To Target", distanceToTarget);

    if((actLaunchAng < (launchAngle + DriveConstants.launchAngError)) && (actLaunchAng > (launchAngle - DriveConstants.launchAngError)) && (rotError < (DriveConstants.maxRotError)) && (rotError > -(DriveConstants.maxRotError))){

      LiveBottom.LiveBottomIn();  // Doesn't stop until B Button isn't pressed
      

    }


  }

  };
//}

  
public void StopLaunch(){

  //return Commands.run(() -> {

  //AutoLaunch.Launch(DriveSubsystem.Goal).cancel();

  Launcher.rotOverRide = false;
  LiveBottom.LiveBottomStop();
  Launcher.LauncherStop();       
  Launcher.rotCmmd = 0 ;
  
 }//)
  //;}



}


  

 /*  public static void EndLaunch(){

    Launcher.LauncherStop();
    LiveBottom.LiveBottomStop();
    Launcher.rotCmmd = 0;
    Launcher.rotOverRide = false;

  }*/







// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//          COMMANDS
// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //public Command AutoLaunchRedCommand = AutoLaunch.Launch(DriveSubsystem.redHub);
  //public Command AutoLaunchBlueCommand = AutoLaunch.Launch(DriveSubsystem.blueHub);          
// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    







