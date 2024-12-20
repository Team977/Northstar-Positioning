package frc.robot.util.Libary;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class TransformFOM {

  private List<FOMSupplier> suppliers = new ArrayList<>();

  // Transform2d newData;

  public void addSupplier(FOMSupplier newData) {
    suppliers.add(newData);
  }

  public Transform2d update() {

    double TotaleFOM = 0;
    Transform2d totatData = new Transform2d(0, 0, new Rotation2d(0));

    for (FOMSupplier instance : suppliers) {

      double InversedFOM = 1 / Math.pow(instance.FOM.get(), 2);

      Transform2d partTransform = instance.Offset.get().times(InversedFOM);

      TotaleFOM += InversedFOM;
      totatData = totatData.plus(partTransform);
    }

    SmartDashboard.putNumber("SUP SIZE", suppliers.size());
    return totatData.div(TotaleFOM);
  }

  /** FOMSupplier */
  public static class FOMSupplier {

    Supplier<Transform2d> Offset;
    Supplier<Double> FOM;
  }
}
