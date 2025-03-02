package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class OverrideSpedClimbArm extends Command {
    private final ClimbSubsystem m_climbSub;
    private final double d_speed;
        
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public OverrideSpedClimbArm(ClimbSubsystem climbSub, double speed) {
        m_climbSub = climbSub;
        d_speed = speed;
        addRequirements(m_climbSub);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climbSub.setArmSpeedOverride(d_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        m_climbSub.setArmSpeedOverride(0);
    }
     
   // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
