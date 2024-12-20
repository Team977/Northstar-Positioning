package frc.robot.util.Libary;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class AvaerageValue {

  List<Double> values = new ArrayList<>(50);
  int Limit;

  double LastIndex = 0;

  DoubleSupplier getter;

  double avg;

  public AvaerageValue(int limit, DoubleSupplier getter) {
    Limit = limit;
    this.getter = getter;
  }

  void Update() {
    if (values.size() > Limit) {
      values.remove(Limit);
    }

    values.add(0, getter.getAsDouble());
  }

  double Avg() {

    double Acclumitve = 0;

    for (double value : values) {
      Acclumitve += value;
    }

    avg = Acclumitve / values.size();
    return avg;
  }

  double getPreviusAvg() {
    return avg;
  }
}
