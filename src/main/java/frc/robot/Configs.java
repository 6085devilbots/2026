package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.feedbackSensor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

public final class Configs {

    public static final class IntakeLiftSetup {
        public static final SparkMaxConfig intakeliftConfig = new SparkMaxConfig();

// - - - - - - - Intake Lift Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -    
        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double liftFactor = 1;
                double liftVelocityFeedForward = 0;
    
                intakeliftConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(2);
                intakeliftConfig.absoluteEncoder
                        .positionConversionFactor(liftFactor) // meters
                        .velocityConversionFactor(0.00000001); // meters per second
                intakeliftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.5, 0, 0.1) // p 0.04
                         .outputRange(-0.1, 0.1)
                        .feedForward.kV(liftVelocityFeedForward);
                       
            }
        
    }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





// - - - - - - - Arm Extend Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class ExtnedArmSetup {
        public static final SparkMaxConfig armExtendConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double extendFactor = 1;
                double extendVelocityFeedForward = 0.02;
    
                armExtendConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(45);
                armExtendConfig.encoder
                        .positionConversionFactor(extendFactor) // meters
                        .velocityConversionFactor(extendFactor / 60.0);  // meters per second
                armExtendConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.5, 0, 0)
                         .outputRange(-0.2, 0.2)
                        .feedForward.kV(extendVelocityFeedForward) ;
                       
            }
        

    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





// - - - - - - - Targeting Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class TargetingMotorSetup {
        public static final SparkMaxConfig targetShooterConfig = new SparkMaxConfig();

        
        static  {     
                     // Use module constants to calculate conversion factors and feed forward gain.
                double targetingFactor = 360;
                double targetingVelocityFeedForward = 0;
    
                targetShooterConfig.softLimit
                        .forwardSoftLimitEnabled(true)
                        .forwardSoftLimit(90)
                        .reverseSoftLimitEnabled(true)
                        .reverseSoftLimit(70);
                targetShooterConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(10);
                targetShooterConfig.absoluteEncoder
                        .zeroOffset(0.1079167)
                        .positionConversionFactor(targetingFactor) // meters
                        .inverted(true)
                        .velocityConversionFactor(targetingFactor ); // meters per second
                targetShooterConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.02, 0, 0.0005)
                        .outputRange(-1, 1)
                        .feedForward.kV(targetingVelocityFeedForward) ;
                        
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class intakeLiftMotorSetup2 {
        public static final SparkMaxConfig intakeLiftConfig2 = new SparkMaxConfig();

        
        static  {     
                     // Use module constants to calculate conversion factors and feed forward gain.
                double intakeLiftFactor2 = 360;
                double intakeLiftVelocityFeedForward2 = 0;
    
                intakeLiftConfig2.softLimit
                        .forwardSoftLimitEnabled(true)
                        .forwardSoftLimit(180)
                        .reverseSoftLimitEnabled(true)
                        .reverseSoftLimit(88);
                intakeLiftConfig2
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(4);
                intakeLiftConfig2.absoluteEncoder
                        .zeroOffset(0.02167)
                        .positionConversionFactor(intakeLiftFactor2) // meters
                        .velocityConversionFactor(intakeLiftFactor2 ); // meters per second
                intakeLiftConfig2.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.05, .000001, 0.01)
                        .outputRange(-0.3, 0.45)
                        .feedForward.kV(intakeLiftVelocityFeedForward2) ;
                        
            }
        
    }


// - - - - - - - Intake Lift Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


/*public static final class intakeLiftMotorSetup {
        public static final SparkMaxConfig intakeLiftConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double intakeLiftFactor = 360;
                double intakeLiftVelocityFeedForward = 1;

                 intakeLiftConfig.softLimit
                        .forwardSoftLimitEnabled(true)
                        .forwardSoftLimit(90)
                        .reverseSoftLimitEnabled(true)
                        .reverseSoftLimit(178);
                intakeLiftConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(4);
                intakeLiftConfig.absoluteEncoder
                        .zeroOffset(0.02167)
                        .positionConversionFactor(360); // meters
                        //.velocityConversionFactor(intakeLiftFactor ); // meters per second
                intakeLiftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-0.1, 0.1)
                        .feedForward.kV(intakeLiftVelocityFeedForward) ;
                        
            }
        
    }*/








// - - - - - - - Intake Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class intakeSetup {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double intakeFactor = 1;
                double intakeVelocityFeedForward = 1;
    
                intakeConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                intakeConfig.encoder
                        .positionConversionFactor(intakeFactor) // meters
                        .velocityConversionFactor(intakeFactor / 60.0); // meters per second
                intakeConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                         .outputRange(-1, 1)
                        .feedForward.kV(intakeVelocityFeedForward) ;
                       
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Right Launch Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class launcherRightSetup {
        public static final SparkMaxConfig launcherRightConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double launcherRightFactor = 1;
                double launcherRightVelocityFeedForward = .00286;
    
                launcherRightConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                 launcherRightConfig.encoder
                        .positionConversionFactor(launcherRightFactor) // meters
                        .velocityConversionFactor(launcherRightFactor); // meters per second
                launcherRightConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(.001, 0, .00001)//0.0004
                         .outputRange(0, 1)
                        .feedForward.kV(launcherRightVelocityFeedForward) ;
                       
            }




            
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 






// - - - - - - - Right Launch Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class launcherLeftSetup {
        public static final SparkMaxConfig launcherLeftConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double launcherLeftFactor = 1;
                double launcherLeftVelocityFeedForward = 0.0034;
    
                launcherLeftConfig
                        .idleMode(IdleMode.kCoast)
                        .inverted(true)
                        .smartCurrentLimit(40);
                launcherLeftConfig.encoder
                        .positionConversionFactor(launcherLeftFactor) // meters
                        .velocityConversionFactor(launcherLeftFactor); // meters per second
                        
                launcherLeftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(.001, 0, .0001)
                         .outputRange(0, 1)
                        .feedForward.kV(launcherLeftVelocityFeedForward) ;
                       
            }




            
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Climber Configuration - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class ClimberSetup {
        public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double climbFactor = 1;
                double climbVelocityFeedForward = 1;
    
                climberConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                climberConfig.encoder
                        .positionConversionFactor(climbFactor) // meters
                        .velocityConversionFactor(climbFactor / 60.0); // meters per second
                climberConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                         .outputRange(-1, 1)
                        .feedForward.kV(climbVelocityFeedForward) ;
                       
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 




// - - - - - - - Live Bottom Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class liveBottomSetup {
        public static final SparkMaxConfig liveBottomConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double liveBottomFactor = 1;
                double liveBottomVelocityFeedForward = 0;
    
                liveBottomConfig
                        .idleMode(IdleMode.kCoast)
                        .inverted(true)
                        .smartCurrentLimit(15);
                liveBottomConfig.encoder
                        .positionConversionFactor(liveBottomFactor) // meters
                        .velocityConversionFactor(liveBottomFactor); // meters per second
                liveBottomConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                         .outputRange(-1, 1)
                        .feedForward.kV(liveBottomVelocityFeedForward) ;
                       
            }




            
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





// - - - - - - - Drive Motors Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;
                                             // 2
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(35);  // Was 50
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.1, 0, 0.1) //.04
                     .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward) ;
                    

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(35);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.3, 0.0, 0.0) //0.3
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}
