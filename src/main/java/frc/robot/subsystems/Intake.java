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

    
    public static SparkMax m_targetSpark;
    public static RelativeEncoder m_targetEncoder;
    public static SparkClosedLoopController m_targetClosedLoopController;
    



// - - - - - - Arm Motor/SparkMax Setups - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    public Intake(){
    

        // - - - - - Intake Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

         m_intakeSpark = new SparkMax(9, MotorType.kBrushless);
         m_intakeEncoder = m_intakeSpark.getEncoder();
         m_intakeClosedLoopController = m_intakeSpark.getClosedLoopController();
         m_intakeSpark.configure(Configs.intakeSetup.intakeConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
         m_intakeEncoder.setPosition(0);



        // - - - - - Intake Lift Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        m_intakeLiftSpark = new SparkMax(14, MotorType.kBrushless);
        m_intakeLiftEncoder = m_intakeLiftSpark.getAbsoluteEncoder();
        m_intakeliftClosedLoopController = m_intakeLiftSpark.getClosedLoopController();
        m_intakeLiftSpark.configure(Configs.IntakeLiftSetup.intakeliftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        

      

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