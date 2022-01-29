package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase{

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    public Autonomous() {
        m_drivetrain = Drivetrain.getInstance();
        m_drivetrain.setDefaultCommand(new Drive());
    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void setupAutoSelector(){}

    public void setupAutoRoutines(){}

    public Command returnAutonomousCommand() {
        return null;
    }

    private void defineAutoPaths(){}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
