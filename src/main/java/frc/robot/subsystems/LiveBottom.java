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



public class LiveBottom {

    private final SparkMax m_intakeSpark;
    private static RelativeEncoder m_intakeEncoder;
    public static SparkClosedLoopController m_intakeClosedLoopController;

    public LiveBottom(){


        // - - - - - LiveBottom Setup - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

         m_liveBottomSpark = new SparkMax(17, MotorType.kBrushless);
         m_liveBottomEncoder = m_liveBottomSpark.getEncoder();
         m_liveBottomClosedLoopController = m_liveBottomSpark.getClosedLoopController();
         m_liveBottomBottomSpark.configure(Configs.liveBottomSetup.liveBottomConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
         m_liveBottomEncoder.setPosition(0);



    }

    public static void LiveBottomIn(){
    
    m_liveBottomClosedLoopController.setSetpoint(DriveConstants.intakeInSpeed, SparkMax.ControlType.kDutyCycle);  // FIX!!!!


    }

    public static void LiveBottomOut(){
    



    }

}   
