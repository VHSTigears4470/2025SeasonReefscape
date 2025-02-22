package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.KitbotDriveConstants;
import frc.robot.Constants.ModuleConstants;
 
public final class Configs {
    public static final class MAXSwerveModule {
        // Swerve
        // public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        // Kitbot Drive
        public static final SparkMaxConfig frontRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftKitbot = new SparkMaxConfig();

        //Coral Subsystem
        public static final SparkMaxConfig intakeMotorTop = new SparkMaxConfig();
        public static final SparkMaxConfig intakeMotor = new SparkMaxConfig();
        public static final SparkMaxConfig intakeMotorArm= new SparkMaxConfig();

        //AlgaeSubsystem
        public static final SparkMaxConfig algaeTopMotor = new SparkMaxConfig();
        public static final SparkMaxConfig algaeArmMotor = new SparkMaxConfig();

        //ClimbSubsystem
        public static final SparkMaxConfig climbMotor = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double d_drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
                    / ModuleConstants.k_DrivingMotorReduction;
            double d_turningFactor = 2 * Math.PI;
            double d_drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;
               //Encoder Conversion factor
            double d_coralFactor = 1 / 108 * 2 * Math.PI; //TODO: Double Check
            double d_algaeFactor = 0; //TODO: Double Check
            double d_climbFactor = 0; //TODO: Double Check
            
            // drivingConfig
            //         .idleMode(IdleMode.kBrake)
            //         .smartCurrentLimit(50)
            //         .inverted(true);
            // drivingConfig.encoder
            //         .positionConversionFactor(drivingFactor) // meters
            //         .velocityConversionFactor(drivingFactor / 60.0);
            // drivingConfig.closedLoop
            //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //         // These are example gains you may need to them for your own robot!
            //         .pid(0.04, 0, 0)
            //         .velocityFF(drivingVelocityFeedForward)
            //         .outputRange(-1, 1);

            frontRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to (change?) them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            frontLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            rearRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(false);
            rearLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(d_turningFactor) // radians
                    .velocityConversionFactor(d_turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, d_turningFactor);


                double kitBotFactor = 1 / ( 2 * Math.PI * KitbotDriveConstants.k_kitbotWheelRadius * KitbotDriveConstants.k_kitbotGearRatio);

                //right motor no invert sets encoder, limits, and brakes
                frontRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                frontRightKitbot
                    .inverted(false); // ORBB: false | AR: true    


                //left motor no invert sets encoder, limits, and brakes
                frontLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                
                    //right back motor not invert sets encoder, limits, and brakes
                rearRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                rearRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearRightKitbot
                    .follow(KitbotDriveConstants.k_frontRightMotorID, false);

                    //left motor not invert sets encoder, limits, and brakes
                rearLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                rearLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearLeftKitbot
                    .follow(KitbotDriveConstants.k_frontLeftMotorID, false);

                intakeMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO 

                intakeMotorArm
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                intakeMotorArm.encoder
                    .positionConversionFactor(d_coralFactor) //degrees
                    .velocityConversionFactor(d_coralFactor / 60.0); 


                algaeTopMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO

                algaeArmMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                algaeArmMotor.encoder
                    .positionConversionFactor(d_algaeFactor) //degrees
                    .velocityConversionFactor(d_algaeFactor / 60.0); 
                    

                climbMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                climbMotor.encoder
                    .positionConversionFactor(d_climbFactor) //degrees
                    .velocityConversionFactor(d_climbFactor / 60.0); 
        }
    }
}