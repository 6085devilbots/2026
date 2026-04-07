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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Climber {

    
    public static SparkMax m_climberSpark;
    public static RelativeEncoder m_climberEncoder; 
    public static SparkClosedLoopController m_climberClosedLoopController;




public Climber(){
                
            
                    // - - - - - Intake Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            
                     m_climberSpark = new SparkMax(17, MotorType.kBrushless);
                     m_climberEncoder = m_climberSpark.getEncoder();
                     m_climberClosedLoopController = m_climberSpark.getClosedLoopController();
                     m_climberSpark.configure(Configs.ClimberSetup.climberConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
                     m_climberEncoder.setPosition(0);
}

 // - - - - - - - Intake Lift - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            
            public static void climbZeroPos(){
                    
                m_climberClosedLoopController.setSetpoint(DriveConstants.climbZero_Pos, SparkMax.ControlType.kPosition);
            
            }
        
    
            public static void climbUpPos(){
            
                m_climberClosedLoopController.setSetpoint(DriveConstants.climbUp_Pos, SparkMax.ControlType.kPosition);
    
            }

}