// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double k_sparkMaxConversionFactor = 0; //TODO: Update

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double k_MaxSpeedMetersPerSecond = 4.8;
    public static final double k_MaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double k_TrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double k_WheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics k_DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),
        new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),
        new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),
        new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double k_FrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double k_FrontRightChassisAngularOffset = 0;
    public static final double k_BackLeftChassisAngularOffset = Math.PI;
    public static final double k_BackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int k_FrontLeftDrivingCanId = 11;
    public static final int k_RearLeftDrivingCanId = 13;
    public static final int k_FrontRightDrivingCanId = 15;
    public static final int k_RearRightDrivingCanId = 17;

    public static final int k_FrontLeftTurningCanId = 10;
    public static final int k_RearLeftTurningCanId = 12;
    public static final int k_FrontRightTurningCanId = 14;
    public static final int k_RearRightTurningCanId = 16;

    // Motor Names
    public enum MotorLocation {
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_LEFT,
      REAR_RIGHT
    };

    // Pigeon2 Id
    public static final int k_pigeon2Id = 16;

    public static final boolean k_GyroReversed = false;
  }

  public static final class CoralConstants {
    //Change IDS
    public static final int k_topID = 0;
    public static final int k_botID = 1;
    public static final int k_armID = 2;

    //Find correct values
    public static final double k_forwardArmPos = 0;
    public static final double k_backwordArmPos = 0;
    
    //Changes arm states
    //Change voltages
    public static final double k_fastVoltage = 4.0; 
    public static final double k_slowVoltage = 1.5; 
    
    public static enum ARM_STATE {
      FORWARD,
      BACKWARD;
    }
  }

  public static final class KitbotDriveConstants {
    public static final int k_frontRightMotorID = 11;
    public static final int k_frontLeftMotorID = 12;
    public static final int k_rearRightMotorID = 13;
    public static final int k_rearLeftMotorID = 14;

    public static final double k_kitbotWheelRadius = 3;
    public static final double k_kitbotGearRatio = 10.75;
  }

public static final class AlgaeConstants {
    //Change IDS
    public static final int k_algaeTopID = 0;
    public static final int k_algaeArmID = 1;

    //Find correct values
    public static final double k_loweredArmPos = 0;
    public static final double k_centeredArmPos = 5;
    public static final double k_raisedArmPos = 20;
    
    //Find correct voltages
    public static final double k_intakeVoltage = -1; 
    public static final double k_dispenseVoltage = 1; 

    public static enum ALGAE_ARM_STATE {
      RAISED,
      LOWERED,
      CENTERED;
    }    
  }

  public static final class ClimbConstants {
    //TODO: Change IDS
    public static final int k_climbMotorID = 0;

    //Find correct values
    public static final double k_upClimbPos = 90;
    public static final double k_downClimbPos = 45;

    public static enum CLIMB_STATE {
      UP,
      DOWN;
    }
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int k_DrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double k_DrivingMotorFreeSpeedRps = NeoMotorConstants.k_FreeSpeedRpm / 60;
    public static final double k_WheelDiameterMeters = 0.0762;
    public static final double k_WheelCircumferenceMeters = k_WheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double k_DrivingMotorReduction = (45.0 * 22) / (k_DrivingMotorPinionTeeth * 15);
    public static final double k_DriveWheelFreeSpeedRps = (k_DrivingMotorFreeSpeedRps * k_WheelCircumferenceMeters)
        / k_DrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int k_DriverControllerPort = 0;
    public static final double k_DriveDeadband = 0.05;

    public static final int k_driverXAxisInverted = 1;
    public static final int k_driverYAxisInverted = 1;
    public static final int k_driverRotAxisInverted = 1;

    public static final int k_driverAxisY = 1; // Check
    public static final int k_driverAxisX = 0; // Check
    public static final int k_driverAxisRot = 4; // Check
    public static final int k_driverAxisOrientedButton = 1; // Check
  }

  public static final class AutoConstants {
    public static final double k_MaxSpeedMetersPerSecond = 3;
    public static final double k_MaxAccelerationMetersPerSecondSquared = 3;
    public static final double k_MaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double k_MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double k_PXController = 1;
    public static final double k_PYController = 1;
    public static final double k_PThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints k_ThetaControllerConstraints = new TrapezoidProfile.Constraints(
        k_MaxAngularSpeedRadiansPerSecond, k_MaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class TeleConstants {
    public static final double k_MaxSpeedMetersPerSecond = 3;
    public static final double k_MaxAccelerationMetersPerSecondSquared = 3;
    public static final double k_MaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double k_MaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  }

  public static final class NeoMotorConstants {
    public static final double k_FreeSpeedRpm = 5676;
  }

  public static final class DebuggingConstants {
    public static final boolean k_swerveDriveDebug = true;
    public static final boolean k_algeaDebug = true;
    public static final boolean k_intakeDebug = true;
    public static final boolean k_climbDebug = true;
    public static final boolean k_kitbotDriveDebug = true;
  }

  public static final class UsingConstants {
    // Drives
    public static final boolean k_usingSwerveDrive = true;
    public static final boolean k_usingKitbotDrive = false;

    public static final boolean k_usingAlgea = false;
    public static final boolean k_usingInake = false;
    public static final boolean k_usingClimb = false;
  }
}