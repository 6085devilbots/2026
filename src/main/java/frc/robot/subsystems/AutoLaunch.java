package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;

import frc.robot.Constants.DriveConstants;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Intake;
import frc.robot.Vision;
import frc.robot.subsystems.AutoLaunchCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import com.revrobotics.AbsoluteEncoder;


public class AutoLaunch {

    public static void Launch(Pose2d tarGoal){


 Launcher.rotOverRide = true;

  var currentPos = DriveSubsystem.getPose2();
  var currentRot =  DriveSubsystem.canandgyro.getRotation2d(); 
  double kP = 0.005; //P gain must be tuned
  double rotError;

  Intake.m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);
  Intake.m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);

  double distanceToTarget = PhotonUtils.getDistanceToPose(currentPos, tarGoal);
  Rotation2d targetYaw = PhotonUtils.getYawToPose(currentPos,tarGoal);

  double iniLaunchVel = ProjectileTrajectory.calcInitialVelocity(ProjectileTrajectory.avgLaunchVelocity());
   
  SmartDashboard.putNumber("Initial Launch Velocity", iniLaunchVel);
  
  double launchAngle = ProjectileTrajectory.calcLaunchAngle(iniLaunchVel, distanceToTarget, 0.395, 1.524); //DriveConstants.initialHeight



  if(Double.isNaN(launchAngle)){


  SmartDashboard.putNumber("Cal Launch Angle", 0);
  }else{

  SmartDashboard.putNumber("Cal Launch Angle", launchAngle);

    
  }

  SmartDashboard.putNumber("Cal Launch Angle", launchAngle);

  double testAngle = ( 1/360) * launchAngle;
  double testAngle2 = ((0.25) - testAngle);

   SmartDashboard.putNumber("TestAngle", testAngle);
   SmartDashboard.putNumber("TestAngle2", testAngle2);

  double calcLaunchAng = (DriveConstants.startAngle + testAngle2);

    SmartDashboard.putNumber("Wanted Launch Angle", calcLaunchAng);

  //Intake.m_targetClosedLoopController.setSetpoint(calcLaunchAng, SparkMax.ControlType.kPosition);
             
  double actLaunchAng = Intake.m_targetEncoder2.getPosition();

  SmartDashboard.putNumber("Launch Ang Encoder", actLaunchAng);

  

  rotError = targetYaw.minus(currentRot).getDegrees();
  Launcher.rotCmmd = rotError * kP ;
     
  SmartDashboard.putNumber("RotError", calcLaunchAng);
  SmartDashboard.putNumber("TargetYaw", targetYaw.getDegrees());
  SmartDashboard.putNumber("RotCmmd", Launcher.rotCmmd);

  SmartDashboard.putNumber("Distance To Target", distanceToTarget);


  //if((((calcLaunchAng) - (actLaunchAng)) < ((calcLaunchAng) * (0.05))) && (rotError < DriveConstants.maxRotError)) {

  //LiveBottom.LiveBottomIn();

    //}
    



}


}


