package frc.robot.utils;

public class LookupTable {
  double[] keys;
  double[] values;

  public LookupTable(double[] inputs, double[] outputs) {
    keys = inputs;
    values = outputs;
  }

  
  public double get(double input) {
    if (input < keys[0]) {
      return values[0];
    } else if (input > keys[keys.length - 1]) {
      return values[keys.length - 1];
    } else {
      int low_i = 0;
      int up_i = 0;
      for (int i = 0; i < keys.length; i++) {
        up_i = i;
        if (keys[up_i] > input) break;
        low_i = up_i;
      }
      double lowerWeight = 1 - (input - keys[low_i]) / (keys[up_i] - keys[low_i]);
      double upperWeight = 1 - (keys[up_i] - input) / (keys[up_i] - keys[low_i]);
      return lowerWeight * values[low_i] + upperWeight * values[up_i];
    }
  }

  public void update(double add) {
    for (int i = 0; i < values.length; i++) {
      values[i] += add;
    }
  }
}