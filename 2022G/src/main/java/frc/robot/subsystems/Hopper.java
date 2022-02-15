package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private LinearFilter filter;

    private CANSparkMax hopperSystem;

    // Define sensors for the hopper to count cargo
    private DigitalInput bottomSensor, topSensor;

    public Hopper() {
        hopperSystem = new CANSparkMax(RobotMap.MOTOR_HOPPER, MotorType.kBrushless);
        hopperSystem.setIdleMode(IdleMode.kBrake);
        hopperSystem.setSmartCurrentLimit(Constants.MAX_HOPPER_BELT_CURRENT);
        filter = LinearFilter.singlePoleIIR(0.2, 0.02);

        bottomSensor = new DigitalInput(1);
        topSensor = new DigitalInput(0);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        putValuesSmartDashboard();
      }

    public static Hopper getInstance() {
        if (hopper == null) {
            hopper = new Hopper();
        }

        return hopper;
    }

    public void runHopper(double speed) {
        hopperSystem.set(-speed); // the speed input needs a multiplier
    }

    public void stopHopper() {
        runHopper(0);
    }

    public void reverseHopper(double speed) {
        runHopper(-speed);
    }

    public double getHopperSpeed() {
        return hopperSystem.get();
    }

    public boolean sensesBallBottom() {
        boolean filteredInput = false;
        if(!bottomSensor.get()){
            double x = filter.calculate(1);
            SmartDashboard.putNumber("x", x);
            filteredInput = x > SmartDashboard.getNumber("lower sensor input threshold", 0.99);
        } else {
            double x = filter.calculate(0);
            SmartDashboard.putNumber("x", x);
            filteredInput = x > SmartDashboard.getNumber("lower sensor input threshold", 0.99);
        }
        SmartDashboard.putBoolean("filteredInput", filteredInput);
        return filteredInput;
    }

    public boolean sensesBallTop() {
        return !topSensor.get();
    }

    public void putSmartDashboardOverrides() {
        SmartDashboard.putNumber("OR: Hopper speed", 0);
    }

    public void updateHopperFromDashboard() {
        runHopper(SmartDashboard.getNumber("OR: Hopper speed", 0));
    }

    public void putValuesSmartDashboard(){
        SmartDashboard.putNumber("Hopper speed", getHopperSpeed());
        SmartDashboard.putBoolean("Lower sensor", sensesBallBottom());
        SmartDashboard.putBoolean("Upper sensor", sensesBallTop());
    }
}
