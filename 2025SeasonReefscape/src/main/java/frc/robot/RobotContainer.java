// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;
import frc.robot.commands.AlgaeCommands.RetractAlgaeArm;
import frc.robot.commands.AlgaeCommands.ShootAlgae;
import frc.robot.commands.ClimbCommands.ExtendClimbArm;
import frc.robot.commands.ClimbCommands.RetractClimbArm;
import frc.robot.commands.CommandGroups.IntakeExtendAlgae;
import frc.robot.commands.CoralCommands.IntakeCoral;
import frc.robot.commands.CoralCommands.ShootCoralFast;
import frc.robot.commands.CoralCommands.ShootCoralSlow;
import frc.robot.commands.CoralCommands.ToggleCoralArm;
import frc.robot.commands.KitbotDrivetrain.ArcadeDrive;
import frc.robot.commands.KitbotDrivetrain.MoveDistance;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.KitbotDriveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private DriveSubsystem m_driveSub;
  private KitbotDriveSubsystem m_kitbotDriveSub;
  private CoralSubsystem m_coralSub;
  private AlgaeSubsystem m_algaeSub;
  private ClimbSubsystem m_climbSub;
  
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
        if(OperatingConstants.k_usingSwerveDrive) {
                m_driveSub = new DriveSubsystem();
                m_driveSub.setDefaultCommand(new RunCommand(
                        () -> m_driveSub.drive(
                                OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), 0), 
                                OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), 0), 
                                OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), 0), 
                                true,
                                "Default / Field Oriented"
                        ), 
                        m_driveSub)
                );
                m_kitbotDriveSub = null;
        } else if(OperatingConstants.k_usingKitbotDrive) { // else if because we should only have one drive at once
                m_kitbotDriveSub = new KitbotDriveSubsystem();
                m_kitbotDriveSub.setDefaultCommand(new ArcadeDrive(
                        m_kitbotDriveSub,
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisX),
                        () -> m_driverController.getRawAxis(OIConstants.k_driverAxisY))
                );
                m_driveSub = null;
        } else {
                m_driveSub = null;
                m_kitbotDriveSub = null;
        }

        if(OperatingConstants.k_usingAlgae) {
                m_algaeSub = new AlgaeSubsystem();
        } else {
                m_algaeSub = null;
        }

        if(OperatingConstants.k_usingClimb) {
                m_climbSub = new ClimbSubsystem();
        } else {
                m_climbSub = null;
        }

        if(OperatingConstants.k_usingCoral) {
                m_coralSub = new CoralSubsystem();
        } else {
                m_coralSub = null;
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
    int preset = 1;
    switch (preset) {
            case 0: 
                    controllerPresetMain(); // Competition Configs
                    break;
            case 1:
                    controllerPresetOne(); // Swerve
                    break;
            case 2:
                        controllerPresetTwo(); // Kitbot
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
        if(OperatingConstants.k_usingSwerveDrive) {
                return new PathPlannerAuto("Straight Auto");
        }
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
        if(OperatingConstants.k_usingAlgae){
        m_driverController.y().onTrue(new IntakeExtendAlgae(m_algaeSub));
        }
        //B-Button
        if(OperatingConstants.k_usingAlgae){
        m_driverController.b().whileTrue(new RetractAlgaeArm(m_algaeSub));
        }
        //X-Button
        if(OperatingConstants.k_usingCoral){
        m_driverController.x().onTrue(new ToggleCoralArm(m_coralSub, CORAL_ARM_STATE.BACKWARD));
        }
        //A-Button
        if(OperatingConstants.k_usingAlgae){
        m_driverController.a().onTrue(new ShootAlgae(m_algaeSub));
        }
        //LB
        if(OperatingConstants.k_usingCoral){
        m_driverController.leftBumper().onTrue(new ShootCoralFast(m_coralSub));
        }
        //LT
        if(OperatingConstants.k_usingCoral){
        m_driverController.leftTrigger().onTrue(new ShootCoralSlow(m_coralSub));
        }
        //RB
        if(OperatingConstants.k_usingClimb){
        m_driverController.rightBumper().whileTrue(new ExtendClimbArm(m_climbSub));
        }
        //RT
        if(OperatingConstants.k_usingClimb){
        m_driverController.rightTrigger().whileTrue(new RetractClimbArm(m_climbSub));
        }
        //D-Pad 
        if(OperatingConstants.k_usingCoral){
        m_driverController.povUp().whileTrue(new IntakeCoral(m_coralSub));//chack validity later
        }
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
        if(OperatingConstants.k_usingSwerveDrive) {
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
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        0.0, 
                                        0.0, 
                                        0.0, 
                                        false,
                                        "Wheels Straight"
                                ), 
                                m_driveSub
                        )
                );


                // drive with Robot Orientation
                m_driverController.rightTrigger().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), 0), 
                                        OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), 0), 
                                        OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), 0), 
                                        true,
                                        "Robot Orientated"
                                ), 
                                m_driveSub
                        )
                );

                // drive forward only
                m_driverController.rightBumper().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        0.4, 
                                        0.0, 
                                        0.0, 
                                        false,
                                        "Forward"
                                ), 
                                m_driveSub
                        )
                );

                // rotate clockwise with joystick input
                m_driverController.leftBumper().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        0.0, 
                                        0.0, 
                                        0.4, 
                                        false,
                                        "Rotate ClockWise"
                                ), 
                                m_driveSub
                        )
                );

                // rotate counter clockwise with joystick input
                m_driverController.leftTrigger().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        0.0, 
                                        0.0, 
                                        -0.4, 
                                        false,
                                        "Rotate CounterClockWise"
                                ), 
                                m_driveSub
                        )
                );
        }
  }

  /**
   * Preset for kitbot
   * Does the following: 
   *    Button A : Drive robot forwards 5 meters
   */
  public void controllerPresetTwo() {
        if(OperatingConstants.k_usingKitbotDrive) {
                m_driverController.a().whileTrue(new MoveDistance(m_kitbotDriveSub, 5, true));
        }
  }
}
