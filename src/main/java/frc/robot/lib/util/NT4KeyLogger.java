package frc.robot.lib.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NT4KeyLogger {
    private final NetworkTable table;
    private final String path;
    public NT4KeyLogger(String path) {
        table = NetworkTableInstance.getDefault().getTable(path);
        this.path = path;
    }

    public void recordValues() { 
        for (String key : table.getKeys()) {
            String t = table.getValue(key).getType().getValueStr();
            switch (table.getValue(key).getType()) {
                case kBoolean:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getBoolean());
                    break;

                case kDouble:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getDouble());
                    break;

                case kString:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getString());
                    break;

                case kRaw:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getRaw());
                    break;
                
                case kBooleanArray:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getBooleanArray());
                    break;

                case kDoubleArray:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getDoubleArray());
                    break;

                case kStringArray:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getStringArray());
                    break;

                case kFloat:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getFloat());
                    break;

                case kInteger:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getInteger());
                    break;
                case kIntegerArray:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getIntegerArray());
                    break;

                case kFloatArray:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getFloatArray());
                    break;

                case kUnassigned:
                    Logger.recordOutput(path + "/" + key, table.getValue(key).getRaw());
                    break;

                default:
                    break;

            }
        }
    }
}