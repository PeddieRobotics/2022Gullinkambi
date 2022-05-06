package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;

public class Logging extends SubsystemBase{

    private int cycleCount;
    private static Logging logging;
    private IntegerLogEntry LogCycles; //just used as an example
    private StringLogEntry commandLog;

    public Logging() {
        DataLogManager.start("","",10);
        DataLogManager.logNetworkTables(false); //DO NOT REMOVE EVER
        DataLog log = DataLogManager.getLog();

        LogCycles = new IntegerLogEntry(log, "/test/Cycles");
        commandLog = new StringLogEntry(log, "/test/Commands");
    }

    public static Logging getInstance(){
        if(logging == null){
            logging = new Logging();
        }
        return logging;
      }

    public void startlogging(){
    }

    public void log(String command){
        commandLog.append(command);
    }

    @Override
    public void periodic() {
        cycleCount++;
        LogCycles.append(cycleCount);
    }
}