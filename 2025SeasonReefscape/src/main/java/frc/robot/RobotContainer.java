// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Constants.UsingConstants;
import frc.robot.commands.KitbotDrivetrain.ArcadeDrive;
import frc.robot.commands.SwerveDrivetrain.SwerveJoystickCommand;
import frc.robot.commands.SwerveDrivetrain.TestDrivingMotors;
import frc.robot.commands.SwerveDrivetrain.TestSetPosCommand;
import frc.robot.commands.SwerveDrivetrain.TestTurningMotors;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.KitbotDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DriveSubsystem m_driveSub;
  KitbotDriveSubsystem m_kitbotDriveSub;
  private final CoralSubsystem m_coralSub = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSub = new AlgaeSubsystem();
  private final ClimbSubsystem m_climbSub = new ClimbSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.k_DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsystems() {
        if(UsingConstants.k_usingSwerveDrive) {
                m_driveSub = new DriveSubsystem();
                m_driveSub.setDefaultCommand(new SwerveJoystickCommand(
                        m_driveSub, 
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisY),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisX),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisRot),
                        () -> true,
                        "Default / Field Oriented"
                )
                );
        } else if(UsingConstants.k_usingKitbotDrive) { // else if because we should only have one drive at once
                m_kitbotDriveSub = new KitbotDriveSubsystem();
                m_kitbotDriveSub.setDefaultCommand(new ArcadeDrive(
                        m_kitbotDriveSub,
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisX),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisRot))
                );
        }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    int preset = 2;
    switch (preset) {
            case 0: 
                    controllerPresetMain();
                    break;
            case 1:
                    controllerPresetOne();
                    break;
            default:
                    controllerPresetMain();
                    break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  // Presets
  /**
   * The default / main preset used for comps
   * Does the following: 
   *    Left Joystick : Swerve (XY Translation)
   *    Right Joystick :  Swerve (Heading)
   *    Y : Intake/Extend Algae
   *    B : Retract Algae
   *    X : Toggle Coral Arm
   *    A :  Shoot Algae
   *    LB:  Fast Shoot
   *    LT:  Slow Shoot
   *    RB: Climb Extend
   *    RT: Climb Retract
   *    D-Pad: Intake Coral
   */
  public void controllerPresetMain() { //subject to change (while/on true)
        //Swerve stuff goes here
        //Left Joystick
        
        //Right Joystick
        
        //Y-button
        m_driverController.y().onTrue(new IntakeExtendAlgae(m_algaeSub));
        //B-Button
        m_driverController.b().whileTrue(new RetractAlgaeArm(m_algaeSub));
        //X-Button
        m_driverController.x().onTrue(newToggleCoralArm(m_coralSub));
        //A-Button
        m_driverController.a().whileTrue(new ShootAlgae(m_algaeSub));
        //LB
        m_driverController.leftBumber().whileTrue(new ShootCoralFast(m_coralSub));
        //LT
        m_driverController.leftTrigger().whileTrue(new ShootCoralSlow(m_coralSub));
        //RB
        m_driverController.rightBumper().whileTrue(new ExtendClimbArm(m_climbSub));
        //RT
        m_driverController.rightTrigger().whileTrue(new RetractClimbArm(m_climbSub));
        //D-Pad 
        m_driverController.povAny().whileTrue(new IntakeCoral(m_coralSub));//chack validity later
  }

  /**
   * The default / main preset used for comps
   * Does the following: 
   *    Button B : Reset Swerve's Odometer 
   *    Button A : Stops All Swerve Modules
   *    Button X : Make Wheels Straight
   *    Right Trigger : Hold + Joystick w/o being field orientated
   *    Right Bumper : Hold to go forward relative to robot
   *    Left Trigger : Rotate Clckwise
   *    Left Bumper : Rotate Counter Clockwise
   */
  public void controllerPresetOne() {
        // Reset Odom
        m_driverController.b().onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                ()-> m_driveSub.zeroHeading(),
                                m_driveSub
                        ),
                        new InstantCommand(
                                ()-> m_driveSub.resetOdometry(new Pose2d()),
                                m_driveSub
                        )
                )
        );

        // Stop Button
        m_driverController.a().onTrue(
                new InstantCommand(
                        () -> m_driveSub.stopModules(), 
                        m_driveSub
                )
        );

        // Straighten Wheels
        m_driverController.x().whileTrue(
                new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (boolean) false,
                        "Wheels Straight"
                )
        );


        // drive with Robot Orientation
        m_driverController.rightTrigger().whileTrue(
                new SwerveJoystickCommand(
                        m_driveSub,
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisY),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisX),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisRot),
                        // When pressed, changes to robot orientated
                        () -> false,
                        "Robot Orientated"
                )
        );

        // drive forward only
        m_driverController.rightBumper().whileTrue(
                new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.4,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (boolean) false,
                        "Forward"
                )
        );

        // rotate clockwise with joystick input
        m_driverController.leftBumper().whileTrue(
                new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (double) 0.4,
                        () -> (boolean) false,
                        "Rotate"
                )
        );

        // rotate counter clockwise with joystick input
        m_driverController.leftTrigger().whileTrue(
                new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (double) -0.4,
                        () -> (boolean) false,
                        "Rotate"
                )
        );

  }

  /**
   * The default / main preset used for comps
   * Does the following: 
   *    Right Trigger : Reset Heading & Odom
   *    Right Bumper : Rotate Wheels Clockwise
   *    Left Bumper : Rotate Wheels Clockwise
   * 
   *    Button B : Set Wheels Right
   *    Button X : Set Wheels Left
   *    Button Y : Set Wheel Straight
   *    Button A : Set Wheels Backward
   *    
   *    Left Trigger + Button B : Move Robot Right
   *    Left Trigger + Button X : Move Robot Left
   *    Left Trigger + Button Y : Move Robot Forward
   *    Left Trigger + Button A : Move Robot Backward
   * 
   *    Start Button : Move Wheels In Whatever Direction they're facing at 10% max speed
   */
  public void controllerPresetTwo() {
        // Reset odom, reset encoders, go to 0 pos (turn), turn, 
        // Reset pos = > zero gyro -> reset odom
        // turn (idk why )
        m_driverController.rightTrigger().onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                ()-> m_driveSub.zeroHeading(),
                                m_driveSub
                        ),
                        new InstantCommand(
                                ()-> m_driveSub.resetOdometry(new Pose2d()),
                                m_driveSub
                        )
                )
        );

        // right movement
        m_driverController.b().and(m_driverController.leftTrigger()).whileTrue(
                    new SwerveJoystickCommand(
                    m_driveSub,
                    () -> (double) 0.0,
                    () -> (double) 0.4,
                    () -> (double) 0.0,
                    () -> (boolean) false,
                    "Right Movement"
                )
        );

        // right rotation
        m_driverController.b().and(m_driverController.leftTrigger().negate()).whileTrue(
                new TestSetPosCommand(m_driveSub, Math.PI * 0.5)
        );

        // left movement
        m_driverController.x().and(m_driverController.leftTrigger()).whileTrue(
                        new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.0,
                        () -> (double) -0.4,
                        () -> (double) 0.0,
                        () -> (boolean) false,
                        "Left Movement"
                )
        );

        // left rotation
        m_driverController.x().and(m_driverController.leftTrigger().negate()).whileTrue(
                new TestSetPosCommand(m_driveSub, Math.PI * 1.5)
        );

        // forwards movement
        m_driverController.y().and(m_driverController.leftTrigger()).whileTrue(
                        new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) 0.4,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (boolean) false,
                        "Forward Movement"
                )
        );

        // forwards rotation
        m_driverController.y().and(m_driverController.leftTrigger().negate()).whileTrue(
                new TestSetPosCommand(m_driveSub, 0)
        );

        // backwards movement
        m_driverController.a().and(m_driverController.leftTrigger()).whileTrue(
                        new SwerveJoystickCommand(
                        m_driveSub,
                        () -> (double) -0.4,
                        () -> (double) 0.0,
                        () -> (double) 0.0,
                        () -> (boolean) false,
                        "Backward Movement"
                )
        );
\


        // backwards rotation
        m_driverController.a().and(m_driverController.leftTrigger().negate()).whileTrue(
                new TestSetPosCommand(m_driveSub, Math.PI)
        );

        // Rotate wheels clockwise
        m_driverController.rightBumper().whileTrue(
                new TestTurningMotors(m_driveSub, true)
        );

        // Rotate wheels counterclockwise
        m_driverController.leftBumper().whileTrue(
                new TestTurningMotors(m_driveSub, false)
        );

        m_driverController.start().whileTrue(
                new TestDrivingMotors(m_driveSub, 0.10 * TeleConstants.k_MaxSpeedMetersPerSecond)
        );
  }
}
