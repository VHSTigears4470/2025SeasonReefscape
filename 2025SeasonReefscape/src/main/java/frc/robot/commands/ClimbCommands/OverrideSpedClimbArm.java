package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class OverrideSpedClimbArm extends Command {
    private final ClimbSubsystem m_climbSub;
    private double d_speed;
    private String m_stringMessage;
        
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public OverrideSpedClimbArm(ClimbSubsystem climbSub, double speed, String message) {
        m_climbSub = climbSub;
        d_speed = speed;
        m_stringMessage = message;
        SmartDashboard.putNumber(m_stringMessage, d_speed);
        addRequirements(m_climbSub);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Commanding Climb Subsystem", m_stringMessage);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        d_speed = SmartDashboard.getNumber(m_stringMessage, d_speed);
        if(Math.abs(d_speed) > 1) {
            d_speed = Math.signum(d_speed) * 1;
            SmartDashboard.putNumber(m_stringMessage, d_speed);
        }
        m_climbSub.setArmSpeedOverride(d_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        SmartDashboard.putString("Commanding Climb Subsystem", m_stringMessage);
        m_climbSub.setArmSpeedOverride(0);
    }
     
   // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
