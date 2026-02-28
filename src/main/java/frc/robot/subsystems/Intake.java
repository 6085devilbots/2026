package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;



public class Intake{

    public static SparkMax m_intakeLiftSpark;
    public static AbsoluteEncoder m_intakeLiftEncoder; 
    public static SparkClosedLoopController m_intakeliftClosedLoopController;

    private final SparkMax m_intakeSpark;
    private static RelativeEncoder m_intakeEncoder;
    public static SparkClosedLoopController m_intakeClosedLoopController;

    public static SparkMax m_launcherSpark12;
    public static RelativeEncoder m_launcherEncoder12;
    public static SparkClosedLoopController m_launcherClosedLoopController12;

    public static SparkMax m_launcherSpark13;
    public static RelativeEncoder m_launcherEncoder13;
    public static SparkClosedLoopController m_launcherClosedLoopController13;

  
    public static SparkMax m_targetSpark;
    public static RelativeEncoder m_targetEncoder;
    public static SparkClosedLoopController m_targetClosedLoopController;
    



// - - - - - - Arm Motor/SparkMax Setups - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    public Intake(){
    
        m_intakeLiftSpark = new SparkMax(14, MotorType.kBrushless);
        m_intakeLiftEncoder = m_intakeLiftSpark.getAbsoluteEncoder();
        m_intakeliftClosedLoopController = m_intakeLiftSpark.getClosedLoopController();
        m_intakeLiftSpark.configure(Configs.IntakeLiftSetup.intakeliftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        

      
         // - - - - - Intake Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

         m_intakeSpark = new SparkMax(9, MotorType.kBrushless);
         m_intakeEncoder = m_intakeSpark.getEncoder();
         m_intakeClosedLoopController = m_intakeSpark.getClosedLoopController();
         m_intakeSpark.configure(Configs.intakeSetup.intakeConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
         m_intakeEncoder.setPosition(0);

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
       

        // - - - - - Targeting Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

         m_targetSpark = new SparkMax(15, MotorType.kBrushless);
         m_targetEncoder = m_targetSpark.getEncoder();
         m_targetClosedLoopController = m_targetSpark.getClosedLoopController();
         m_targetSpark.configure(Configs.TargetingMotorSetup.targetShooterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
         m_targetEncoder.setPosition(0);

        
    }
    
    
    
 






// - - - - - - - Arm Lift - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    public static void intakeDownPos(){
        
        //m_intakeliftClosedLoopController.setSetpoint(DriveConstants.intakeDown_Pos, SparkMax.ControlType.kPosition);

    }
 


    public static void intakeUpPos(){
        
        //m_intakeliftClosedLoopController.setSetpoint(DriveConstants.intakeUp_Pos, SparkMax.ControlType.kPosition);

    }

    public static double intakeLiftPosition(){
        
        return m_intakeLiftEncoder.getPosition();

    }
   
// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    









// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -




    // - - - - - - - Intake - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    public static void IntakeIn(){

        m_intakeClosedLoopController.setSetpoint(DriveConstants.intakeInSpeed, SparkMax.ControlType.kDutyCycle);
     
    }

    public static void IntakeOut(){

        m_intakeClosedLoopController.setSetpoint(DriveConstants.intakeOutSpeed, SparkMax.ControlType.kDutyCycle);
   
    }

    public static void IntakeStop(){

        m_intakeClosedLoopController.setSetpoint(0, SparkMax.ControlType.kDutyCycle);
      
    }
   

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







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

    public static void LauncherStop(){

       m_launcherClosedLoopController12.setSetpoint(0, SparkMax.ControlType.kVelocity);
       m_launcherClosedLoopController13.setSetpoint(0, SparkMax.ControlType.kVelocity);

    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

 // - - - - - - - - - - Targeting  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

     public static void TargetIncrease(){

        m_targetClosedLoopController.setSetpoint(DriveConstants.tarIncreaseAng_Pos, SparkMax.ControlType.kPosition);
        

    }

     public static void TargetDecrease(){

        m_targetClosedLoopController.setSetpoint(DriveConstants.tarDecreaseAng_Pos, SparkMax.ControlType.kPosition);
        

    }
    


     public static double targetPosition(){
        
        return m_targetEncoder.getPosition();
    }



// - - - - - - - - - - - Climb - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


    
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

}