package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

public class UpdateLogs {

    private static Drivetrain m_drivetrain;
    private static Climber m_climber;
    private static Flywheel m_flywheel;
    private static Hopper m_hopper;
    private static Intake m_intake;
    private static Limelight m_limelight;

    private static UpdateLogs instance;

    private UpdateLogs(){
        m_drivetrain = Drivetrain.getInstance();
        m_climber = Climber.getInstance();
        m_flywheel = Flywheel.getInstance();
        m_hopper = Hopper.getInstance();
        m_intake = Intake.getInstance();
        m_limelight = Limelight.getInstance();
    }

    public static UpdateLogs getInstance(){
        if (instance == null){
            instance = new UpdateLogs();
        }
        return instance;
    }

    public void updateDrivetrainLogData(){
        double pose[] = new double[3];

        //Encoders
        Logger.getInstance().recordOutput("Drivetrain/LeftEncPosition", m_drivetrain.getLeftEncoderPosition());
        Logger.getInstance().recordOutput("Drivetrain/LeftEncVelocity", m_drivetrain.getLeftEncoderVelocity());
        Logger.getInstance().recordOutput("Drivetrain/RightEncPosition", m_drivetrain.getRightEncoderPosition());
        Logger.getInstance().recordOutput("Drivetrain/RightEncVelocity", m_drivetrain.getRightEncoderVelocity());   
        
        //Pose
        pose[0] = m_drivetrain.getPose().getTranslation().getX();
        pose[1] = m_drivetrain.getPose().getTranslation().getY();
        pose[2] = Rotation2d.fromDegrees(m_drivetrain.getHeading()).getRadians();
        Logger.getInstance().recordOutput("Odometry/PoseArray", pose);

        //Drive Setpoints
        Logger.getInstance().recordOutput("Drivetrain/SpeedSetpoint", m_drivetrain.getSpeedSetpoint());
        Logger.getInstance().recordOutput("Drivetrain/TurnSetpoint", m_drivetrain.getTurnSetpoint());

        //Drive Motor Data
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftMasterVelocity", m_drivetrain.getLeftMasterVelocity());
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftFollowerVelocity", m_drivetrain.getLeftFollowerVelocity());
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftFollower2Velocity", m_drivetrain.getLeftFollower2Velocity());
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/RightMasterVelocity", m_drivetrain.getRightMasterVelocity());
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/RightFollowerVelocity", m_drivetrain.getRightFollowerVelocity());
        Logger.getInstance().recordOutput("Drivetrain/MotorVelocities/RightFollower2Velocity", m_drivetrain.getRightFollower2Velocity());

        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftMasterCurrent", m_drivetrain.getLeftMasterCurrent());
        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftFollowerCurrent", m_drivetrain.getLeftFollowerCurrent());
        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftFollower2Current", m_drivetrain.getLeftFollower2Current());
        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/RightMasterCurrent", m_drivetrain.getRightMasterCurrent());
        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/RightFollowerCurrent", m_drivetrain.getRightFollowerCurrent());
        Logger.getInstance().recordOutput("Drivetrain/MotorCurrents/RightFollower2Current", m_drivetrain.getRightFollower2Current());
        
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftMasterTemperature", m_drivetrain.getLeftMasterMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftFollowerTemperature", m_drivetrain.getLeftFollowerMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftFollower2Temperature", m_drivetrain.getLeftFollower2MotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightMasterTemperature", m_drivetrain.getRightMasterMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightFollowerTemperature", m_drivetrain.getRightFollowerMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightFollower2Temperature", m_drivetrain.getRightFollower2MotorTemperature());

        Logger.getInstance().recordOutput("Drivetrain/HeadingValue", m_drivetrain.getHeading());
    }

    public void updateClimberLogData(){
        Logger.getInstance().recordOutput("CompressorPressure", m_climber.getCompressorPressure());

        //Winch Motor Data
        Logger.getInstance().recordOutput("Climber/PrimaryArmWinchVelocity", m_climber.getPrimaryArmVelocity());
        Logger.getInstance().recordOutput("Climber/SecondaryArmWinchVelocity", m_climber.getSecondaryArmVelocity());

        Logger.getInstance().recordOutput("Climber/PrimaryArmWinchCurrent", m_climber.getPrimaryArmCurrent());
        Logger.getInstance().recordOutput("Climber/SecondaryArmWinchCurrent", m_climber.getSecondaryArmCurrent());
        
        Logger.getInstance().recordOutput("Climber/PrimaryArmWinchTemperature", m_climber.getPrimaryArmMotorTemperature());
        Logger.getInstance().recordOutput("Climber/SecondaryArmWinchTemperature", m_climber.getSecondaryArmMotorTemperature());
    }

    public void updateFlywheelLogData(){
        //Encoders
        Logger.getInstance().recordOutput("Flywheel/FlywheelEncPosition", m_flywheel.getFlywheelEncoderPosition());
        Logger.getInstance().recordOutput("Flywheel/FlywheelEncVelocity", m_flywheel.getFlywheelEncoderVelocity());

        //Flywheel Setpoints
        Logger.getInstance().recordOutput("Flywheel/RPMSetpoint", m_flywheel.getFlywheelSetpoint());

        //Motor Data
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelVelocity", m_flywheel.getPrimaryFlywheelVelocity());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheelVelocity", m_flywheel.getSecondaryFlywheelVelocity());
        
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelCurrent", m_flywheel.getPrimaryFlywheelCurrent());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Current", m_flywheel.getSecondaryFlywheelCurrent());
        
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelTemperature", m_flywheel.getPrimaryFlywheelMotorTemperature());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Temperature", m_flywheel.getSecondaryFlywheelMotorTemperature());
        
        Logger.getInstance().recordOutput("Flywheel/HoodUp", m_flywheel.isHoodUp());
    }

    public void updateHopperLogData(){
        Logger.getInstance().recordOutput("Hopper/BottomSensorValue", m_hopper.sensesBallBottom());
        Logger.getInstance().recordOutput("Hopper/TopSensorValue", m_hopper.sensesBallTop());

        //Hopper Motor Data
        Logger.getInstance().recordOutput("Hopper/HopperMotorVelocity", m_hopper.getHopperVelocity());
        Logger.getInstance().recordOutput("Hopper/HopperMotorCurrent", m_hopper.getHopperCurrent());
        Logger.getInstance().recordOutput("Hopper/HopperMotorTemperature", m_hopper.getHopperMotorTemperature());
    }

    public void updateIntakeLogData(){
        Logger.getInstance().recordOutput("Intake/IntakeSolenoidOn", m_intake.getSolenoidState());

        //Intake Motor Data
        Logger.getInstance().recordOutput("Intake/IntakeMotorVelocity", m_intake.getIntakeVelocity());
        Logger.getInstance().recordOutput("Intake/IntakeMotorCurrent", m_intake.getIntakeCurrent());
        Logger.getInstance().recordOutput("Intake/IntakeMotorTemperature", m_intake.getIntakeMotorTemperature());
    }

    public void updateLimelightLogData(){
        Logger.getInstance().recordOutput("Limelight/TargetHeightPixels", m_limelight.getTvert());
        Logger.getInstance().recordOutput("Limelight/TargetWidthPixels", m_limelight.getThor());
        Logger.getInstance().recordOutput("Limelight/HorizonalOffsetDegrees", m_limelight.getTx());
        Logger.getInstance().recordOutput("Limelight/VerticalOffsetDegrees", m_limelight.getTy());
        Logger.getInstance().recordOutput("Limelight/TargetArea%ofImmues", m_limelight.getTa());
    }
}
