package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.Configs;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.DriveConstants;
//import java.lang.reflect.Type;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;




public class Intake{

    public static SparkMax m_intakeLiftSpark;
    public static AbsoluteEncoder m_intakeLiftEncoder; 
    public static SparkClosedLoopController m_intakeliftClosedLoopController;

    private final SparkMax m_intakeSpark;
    private static RelativeEncoder m_intakeEncoder;
    public static SparkClosedLoopController m_intakeClosedLoopController;

    public static Command intakeDownPos;
    public static Command intakeUpPos;
    public static Command intakeMidPos;
            
            // - - - - - - Intake Motor/SparkMax Setups - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            
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
                     m_intakeLiftSpark.configure(Configs.intakeLiftMotorSetup2.intakeLiftConfig2, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
            
                     
                }
                
            
            // - - - - - - - Intake Lift - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            
            public static void intakeDownPos(){
                    
                m_intakeliftClosedLoopController.setSetpoint(DriveConstants.intakeDown_Pos, SparkMax.ControlType.kPosition);
            
            }
        
            public static void intakeUpPos(){
            
                m_intakeliftClosedLoopController.setSetpoint(DriveConstants.intakeUp_Pos, SparkMax.ControlType.kPosition);
    
            }
    
            public static void intakeMidPos(){
            
                m_intakeliftClosedLoopController.setSetpoint(DriveConstants.intakeMid_Pos, SparkMax.ControlType.kPosition);
    
            }
    
            public static double intakeLiftPosition(){
            
                return m_intakeLiftEncoder.getPosition();
    
            }
    
// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//          COMMANDS
// - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            public Command intakeDownCommand = Intake.intakeDownPos;
            public Command intakeUpCommand = Intake.intakeUpPos;
            public Command intakeMidCommand = Intake.intakeMidPos;
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






}