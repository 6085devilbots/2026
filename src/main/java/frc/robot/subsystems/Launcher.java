package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;


public class Launcher {

        public static SparkMax m_launcherSpark12;
        public static RelativeEncoder m_launcherEncoder12;
        public static SparkClosedLoopController m_launcherClosedLoopController12;
    
        public static SparkMax m_launcherSpark13;
        public static RelativeEncoder m_launcherEncoder13;
        public static SparkClosedLoopController m_launcherClosedLoopController13;
    
        public static SparkMax m_targetSpark;
        public static AbsoluteEncoder m_targetEncoder2;
        public static SparkClosedLoopController m_targetClosedLoopController;
    
        public static boolean rotOverRide;
        public static double rotCmmd;
    
        public static Command LauncherIn;
        public static Command LauncherOut;
        public static Command LauncherStop;
        
        
    
    public Launcher(){
    
    
             // - - - - - Launcher Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
             m_launcherSpark12 = new SparkMax(12, MotorType.kBrushless);
             m_launcherEncoder12 = m_launcherSpark12.getEncoder();
             m_launcherClosedLoopController12 = m_launcherSpark12.getClosedLoopController();
             m_launcherSpark12.configure(Configs.launcherRightSetup.launcherRightConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
             m_launcherEncoder12.setPosition(0);
    
             m_launcherSpark13 = new SparkMax(13, MotorType.kBrushless);
             m_launcherEncoder13 = m_launcherSpark13.getEncoder();
             m_launcherClosedLoopController13 = m_launcherSpark13.getClosedLoopController();
             m_launcherSpark13.configure(Configs.launcherLeftSetup.launcherLeftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
             m_launcherEncoder13.setPosition(0);
             // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   
    
             // - - - - - Targeting Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
             m_targetSpark = new SparkMax(15, MotorType.kBrushless);
             m_targetEncoder2 = m_targetSpark.getAbsoluteEncoder();
             m_targetClosedLoopController = m_targetSpark.getClosedLoopController();
             m_targetSpark.configure(Configs.TargetingMotorSetup.targetShooterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
             // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    
    }
    
    
    
    // - - - - - - - - - - Launcher  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        public static void LauncherIn(){
    
            m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);
            m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);
    
        }
    
    
        public static void LauncherOut(){
    
            m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);
            m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);
            //m_launcherClosedLoopController13.setInverted(true);
    
        }

        public static void LauncherDefault(){
    
            m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherDefaultSpeed, SparkMax.ControlType.kVelocity);
            m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherDefaultSpeed, SparkMax.ControlType.kVelocity);
            //m_launcherClosedLoopController13.setInverted(true);
    
        }
    
        public static void LauncherStop(){
    
           m_launcherClosedLoopController12.setSetpoint(0, SparkMax.ControlType.kVelocity);
           m_launcherClosedLoopController13.setSetpoint(0, SparkMax.ControlType.kVelocity);
    
        }
    
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
        // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        //          COMMANDS
        // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        public Command LauncherInCommand = Launcher.LauncherIn;
        public Command LauncherOutCommand = Launcher.LauncherOut;
        public Command LauncherStopCommand = Launcher.LauncherStop;
        // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // - - - - - - - - - - Targeting  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    public static void TargetIncrease(){

        m_targetClosedLoopController.setSetpoint(DriveConstants.tarIncreaseAng_Pos, SparkMax.ControlType.kPosition);
        
    } 

    public static void TargetDecrease(){

        m_targetClosedLoopController.setSetpoint(DriveConstants.tarDecreaseAng_Pos, SparkMax.ControlType.kPosition);
        
    }

    public static void ManualTarget(double targetDeg){

        m_targetClosedLoopController.setSetpoint(targetDeg, SparkMax.ControlType.kPosition);
        
    }
    
     public static double targetPosition(){
        
       return m_targetEncoder2.getPosition(); 

    }
   
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

   
}




