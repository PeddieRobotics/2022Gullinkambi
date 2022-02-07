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
        Logger.getInstance().recordOutput("Drivetrain/LeftMasterVelocity", m_drivetrain.getLeftMasterVelocity());
        Logger.getInstance().recordOutput("Drivetrain/LeftFollowerVelocity", m_drivetrain.getLeftFollowerVelocity());
        Logger.getInstance().recordOutput("Drivetrain/RightMasterVelocity", m_drivetrain.getRightMasterVelocity());
        Logger.getInstance().recordOutput("Drivetrain/RightFollowerVelocity", m_drivetrain.getRightFollowerVelocity());
        
        Logger.getInstance().recordOutput("Drivetrain/LeftMasterCurrent", m_drivetrain.getLeftMasterCurrent());
        Logger.getInstance().recordOutput("Drivetrain/LeftFollowerCurrent", m_drivetrain.getLeftFollowerCurrent());
        Logger.getInstance().recordOutput("Drivetrain/RightMasterCurrent", m_drivetrain.getRightMasterCurrent());
        Logger.getInstance().recordOutput("Drivetrain/RightFollowerCurrent", m_drivetrain.getRightFollowerCurrent());
        
        Logger.getInstance().recordOutput("Drivetrain/LeftMasterTemperature", m_drivetrain.getLeftMasterMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/LeftFollowerTemperature", m_drivetrain.getLeftFollowerMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/RightMasterTemperature", m_drivetrain.getRightMasterMotorTemperature());
        Logger.getInstance().recordOutput("Drivetrain/RightFollowerTemperature", m_drivetrain.getRightFollowerMotorTemperature());
    }

    public void updateClimberLogData(){
        Logger.getInstance().recordOutput("Climber/LeftSolenoidOn", m_climber.getArmState());
        Logger.getInstance().recordOutput("CompressorPressure", m_climber.getCompressorPressure());
    }

    public void updateFlywheelLogData(){
        //Encoders
        Logger.getInstance().recordOutput("Flywheel/FlywheelEncPosition", m_flywheel.getFlywheelEncoderPosition());
        Logger.getInstance().recordOutput("Flywheel/FlywheelEncVelocity", m_flywheel.getFlywheelEncoderVelocity());

        //Flywheel Setpoints
        Logger.getInstance().recordOutput("Flywheel/RPMSetpoint", m_flywheel.getFlywheelSetpoint());

        //Motor Data
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelVelocity", m_flywheel.getPrimaryflywheelVelocity());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheelVelocity", m_flywheel.getSecondaryflywheelVelocity());
        
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelCurrent", m_flywheel.getPrimaryflywheelCurrent());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Current", m_flywheel.getSecondaryflywheelCurrent());
        
        Logger.getInstance().recordOutput("Flywheel/PrimaryFlywheelTemperature", m_flywheel.getPrimaryflywheelMotorTemperature());
        Logger.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Temperature", m_flywheel.getSecondaryflywheelMotorTemperature());
        
        Logger.getInstance().recordOutput("Flywheel/HoodUp", m_flywheel.isHoodUp());
    }

    public void updateHopperLogData(){
        Logger.getInstance().recordOutput("Hopper/HopperMotorVelocity", m_hopper.getHopperVelocity());

        Logger.getInstance().recordOutput("Hopper/BottomSensorValue", m_hopper.sensesBallBottom());
        Logger.getInstance().recordOutput("Hopper/BottomSensorValue", m_hopper.sensesBallTop());
    }

    public void updateIntakeLogData(){
        Logger.getInstance().recordOutput("Intake/IntakeSolenoidOn", m_intake.getSolenoidState());
        Logger.getInstance().recordOutput("Intake/IntakeMotorVelocity", m_intake.getIntakeVelocity());
    }

    public void updateLimelightLogData(){
        Logger.getInstance().recordOutput("Limelight/HasTarget", m_limelight.getTv());
        Logger.getInstance().recordOutput("Limelight/TargetHeightPixels", m_limelight.getTvert());
        Logger.getInstance().recordOutput("Limelight/TargetWidthPixels", m_limelight.getThor());
        Logger.getInstance().recordOutput("Limelight/HorizonalOffsetDegrees", m_limelight.getTx());
        Logger.getInstance().recordOutput("Limelight/VerticalOffsetDegrees", m_limelight.getTy());
        Logger.getInstance().recordOutput("Limelight/TargetArea%ofImmues", m_limelight.getTa());
    }
}
