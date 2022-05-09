package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class UnjamIntakeWheels extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;
    private Flywheel m_flywheel;
    private boolean unjamOneBall;
    private double initialTime;
    public boolean armed, armedAgain;
    private double speed;

    public UnjamIntakeWheels(double speed) {
        m_intake = Intake.getInstance();
         m_hopper = Hopper.getInstance();
        // m_flywheel = Flywheel.getInstance();
        // addRequirements(m_intake, m_hopper, m_flywheel);
        addRequirements(m_intake, m_hopper);
        this.speed = speed;

        // armed = false;
        // armedAgain = false;
    }

    @Override
    public void initialize() {
        // initialTime = Timer.getFPGATimestamp();

        // m_intake.setIntakeSolenoid(true);
        // m_flywheel.runFlywheelSetpoint(0);
         m_intake.reverseIntake(speed);
        // if(!unjamOneBall){
        //     m_hopper.reverseHopper(Constants.HOPPER_INDEX_POWER);
        // }
        // armed = false;
        // armedAgain = false;
    }

    @Override
    public void execute() {
        // if(unjamOneBall){
        //     if(!armed && m_intake.getIntakeCurrent() > 25.0){
        //         armed = true;
        //     }
        //     if(armed && !armedAgain && m_intake.getIntakeCurrent() < 10.0){
        //         m_hopper.reverseHopper(Constants.HOPPER_INDEX_POWER);
        //         armedAgain = true;
        //     }
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_hopper.stopHopper();
        // armed = false;
        // armedAgain = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    //     if(unjamOneBall){
    //         // Do not allow the robot to try to pop a single ball out for greater than 3 seconds, indicates a different issue...
    //         if(Timer.getFPGATimestamp() - initialTime > 3){
    //             return true;
    //         }
    //         // End the command immediately if the robot is actually empty so we do not get stuck into an unjamming loop
    //         if(!armed && !armedAgain && !m_hopper.sensesBallTop() && !m_hopper.sensesBallBottom()){
    //             return true;
    //         }
    //         // If we detect a spike in the intake current again, stop unjamming since we've gotten rid of one ball
    //         if(armedAgain && m_intake.getIntakeCurrent() > 10.0){
    //             return true;
    //         }
    //     }
    //     return false;
    // }
}
}