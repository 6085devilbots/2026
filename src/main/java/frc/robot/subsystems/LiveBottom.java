package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;


public class LiveBottom {

        public final SparkMax m_liveBottomSpark;
        public static RelativeEncoder m_liveBottomEncoder;
        public static SparkClosedLoopController m_liveBottomClosedLoopController;
    
        public static Command LiveBottomIn;
        public static Command LiveBottomOut;
        public static Command LiveBottomStop;
    
        
    
        public LiveBottom(){
    
    
            // - - - - - LiveBottom Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    
             m_liveBottomSpark = new SparkMax(7, MotorType.kBrushless);
             m_liveBottomEncoder = m_liveBottomSpark.getEncoder();
             m_liveBottomClosedLoopController = m_liveBottomSpark.getClosedLoopController();
             m_liveBottomSpark.configure(Configs.liveBottomSetup.liveBottomConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
             m_liveBottomEncoder.setPosition(0);
    
    
    
        }
    
    
        public static void LiveBottomIn(){
        
          m_liveBottomClosedLoopController.setSetpoint(DriveConstants.liveBottomInSpeed, SparkMax.ControlType.kDutyCycle);  
    
    
        }
    
        public static void LiveBottomOut(){
        
          m_liveBottomClosedLoopController.setSetpoint(DriveConstants.liveBottomOutSpeed, SparkMax.ControlType.kDutyCycle);  
    
    
        }
    
        public static void LiveBottomStop(){
        
          m_liveBottomClosedLoopController.setSetpoint(0, SparkMax.ControlType.kDutyCycle);  
    
    
        }
    
        // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        //          COMMANDS
        // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        public Command LiveBottomInCommand = LiveBottom.LiveBottomIn;
        public Command intakeUpCommand = LiveBottom.LiveBottomOut;
        public Command intakeMidCommand = LiveBottom.LiveBottomStop;
    // - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    


}   
