package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.utils.Constants;


public class Autonomous extends SubsystemBase{

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;
    private final TrajectoryConfig configForward;
    private final TrajectoryConfig configBackwards;
    private final DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private Trajectory test2, straightPathTestHandwritten, straightPathTest, moveForwards, sPathForward, turnInPlace, shoot2High_Layup_B, moveBackwards, test;

    public Autonomous() {
        m_drivetrain = Drivetrain.getInstance();
        m_drivetrain.setDefaultCommand(new Drive());

        autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
            10);

    
        configForward = 
            new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    
        configBackwards = 
            new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            // Reverse the motors
            .setReversed(true);
        
        defineAutoPaths();
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
         //return createCommandFromTrajectory(moveForwards);
        //return createCommandFromTrajectory(sPathForward);
        return createCommandFromTrajectory(straightPathTestHandwritten);
        // return createCommandFromTrajectory(turnInPlace);
        // return createCommandFromTrajectory(shoot2High_Layup_B);
        // return createCommandFromTrajectory(moveBackwards);
        //System.out.println(test.toString());
        // return createCommandFromTrajectory(test);
        // return createCommandFromTrajectory(straightPathTest);
        //return createCommandFromTrajectory(moveBackwards);
        //return createCommandFromTrajectory(Test);
        //return createCommandFromTrajectory(straightPathTest);
    }

    private void defineAutoPaths(){

        test = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            List.of(new Translation2d(1.5, 0.5)),
            new Pose2d(3, 0, new Rotation2d(Math.toRadians(90))),
            // Pass config12
            configForward
        );
        // test = getTransformedTrajectory(test);

        moveForwards = 
            TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.5, 0)),
            // End  meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(Math.toRadians(0))),
            // Pass config
            configForward);

        moveBackwards = 
            TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-0.5, 0)),
            // End  meters straight ahead of where we started, facing forward
            new Pose2d(-1, 0, new Rotation2d(Math.toRadians(0))),
            // Pass config
            configBackwards);
        
        sPathForward = 
            TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(
                  new Translation2d(1, 1),
                  new Translation2d(1.5, 0),
                  new Translation2d(2, -1)
              ),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              // Pass config
              configForward
          );


          straightPathTestHandwritten = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0,0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(1.5, 0.5)
                ), 
                new Pose2d(3, 0, new Rotation2d(0)), 
                
                configForward);

        turnInPlace = 
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(.75, 0)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(1, 0, new Rotation2d(90)),
              // Pass config
              configForward
          );
        
       
        
        shoot2High_Layup_B = getTransformedTrajectory(PathPlanner.loadPath("Shoot2High-Layup-B", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));

        

        straightPathTest = getTransformedTrajectory(PathPlanner.loadPath("StraightPathTest", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
    }

    public SplitFFRamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        RamseteController m_ramseteController = new RamseteController();
        // m_ramseteController.setEnabled(false);
        var leftController = new PIDController(Constants.kPDriveVel, 0, 0);
        var rightController = new PIDController(Constants.kPDriveVel, 0, 0);
        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftReference = table.getEntry("left_reference");
        var leftMeasurement = table.getEntry("left_measurement");
        var rightReference = table.getEntry("right_reference");
        var rightMeasurement = table.getEntry("right_measurement");

        SplitFFRamseteCommand autoCommand  =
          new SplitFFRamseteCommand(
              trajectory,
              m_drivetrain::getPose,
            //   new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                m_ramseteController, //NEED TO CHANGE BACK TO: new RamseteController(),
              new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
              m_drivetrain::getWheelSpeeds,
            //   new PIDController(SmartDashboard.getNumber("KPDriveVel", 0), 0, 0),
            //   new PIDController(SmartDashboard.getNumber("KPDriveVel", 0), 0, 0),
            leftController,
            rightController,
              // RamseteCommand passes volts to the callback
              //m_drivetrain::tankDriveVolts,
              (leftVolts, rightVolts) -> {
                  m_drivetrain.tankDriveVolts(leftVolts, rightVolts);
                  leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                  leftReference.setNumber(leftController.getSetpoint());

                  rightMeasurement.setNumber(-m_drivetrain.getWheelSpeeds().rightMetersPerSecond); //PUT A NEGATIVE HERE, REMINDER TO FIX 2/7
                  rightReference.setNumber(rightController.getSetpoint());
              },
              m_drivetrain);
        return autoCommand;
      }
    
    public Trajectory getTransformedTrajectory(Trajectory t){
        Pose2d newOrigin = new Pose2d(0, 0, Rotation2d.fromDegrees(30));
        Trajectory transformed = t.relativeTo(newOrigin);
        // Transform2d transform = new Pose2d(0,0, Rotation2d.fromDegrees(0)).minus(t.getInitialPose());
        // Trajectory transformed = t.transformBy(transform);
        return transformed;
      }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
