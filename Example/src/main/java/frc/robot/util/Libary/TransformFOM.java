package frc.robot.util.Libary;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class TransformFOM {

  private List<Supplier<FOMSupplier>> suppliers = new ArrayList<>();

  // Transform2d newData;

  public void addSupplier(Supplier<FOMSupplier> newData) {
    suppliers.add(newData);
  }

  public Transform2d update() {

    double TotaleFOM = 1;
    Transform2d totatData = new Transform2d(0, 0, new Rotation2d(0));

    for (Supplier<FOMSupplier> instance : suppliers) {

      FOMSupplier newFOMSupplier = instance.get();

      double InversedFOM = 1 / Math.pow(newFOMSupplier.FOM, 2);

      Transform2d partTransform = newFOMSupplier.Data.times(InversedFOM);

      TotaleFOM += InversedFOM;
      totatData = totatData.plus(partTransform);
    }

    SmartDashboard.putNumber("SUP SIZE", suppliers.size());
    return totatData.div(TotaleFOM);
  }

  /** FOMSupplier */
  public static class FOMSupplier {

    Transform2d Data;
    double FOM;
  }
}
