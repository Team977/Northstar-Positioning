package frc.robot.util.Libary;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PoseTracker {

  public static interface FOMDataIO {

    public default void addFOM(DoubleSupplier fOMdataSuppliers) {}

    public default void DontTust() {}

    public default void setBaseFOM(double FOM) {}

    public default double getFOM() {
      return 0;
    }

    public default Pose2d getPose() {
      return new Pose2d();
    }
  }

  public static class FOMdataOdymetry implements FOMDataIO {

    Supplier<Pose2d> pose;
    double BaseFOM = 0;

    public FOMdataOdymetry(Supplier<Pose2d> pose, double baseFOM) {
      this.pose = pose;
      BaseFOM = baseFOM;
    }

    List<DoubleSupplier> FOMMdataSuppliers = new ArrayList<>();

    public void addFOM(DoubleSupplier fOMdataSuppliers) {
      FOMMdataSuppliers.add(fOMdataSuppliers);
    }

    public void DontTust() {
      BaseFOM = Double.MAX_VALUE;
    }

    public void setBaseFOM(double FOM) {
      BaseFOM = FOM;
    }

    public double getFOM() {
      return CalalateFOM();
    }

    private double CalalateFOM() {
      double FOM = BaseFOM;

      if (FOM == Double.MAX_VALUE) {
        return FOM;
      }

      for (DoubleSupplier FOMdata : FOMMdataSuppliers) {
        FOM += FOMdata.getAsDouble();
      }

      return FOM;
    }

    public Pose2d getPose() {
      return pose.get();
    }
  }

  public static Pose2d pose;

  public static List<Supplier<FOMDataIO>> fomSuppliers;

  public static boolean UseInbuitOdymetryTracker;

  public static Pose2d getPose() {
    return pose;
  }

  public static void setPose(Pose2d pose) {
    PoseTracker.pose = pose;
  }

  public static void addFOMSupllier(Supplier<FOMDataIO> FOMData) {
    fomSuppliers.add(FOMData);
  }

  public static Pose2d claclateNewPose() {
    double acclutiveFOM = 0;

    Pose2d acclulativePose = new Pose2d();

    for (Supplier<FOMDataIO> fomSupplier : fomSuppliers) {
      FOMDataIO fomData = fomSupplier.get();
      double inversedFOM = 1 / Math.pow(fomData.getFOM(), 2);

      Pose2d MagnitudePose = fomData.getPose().times(inversedFOM);
      acclulativePose =
          acclulativePose.plus(
              new Transform2d(
                  MagnitudePose.getX(), MagnitudePose.getY(), MagnitudePose.getRotation()));

      acclutiveFOM += inversedFOM;
    }

    return acclulativePose.div(acclutiveFOM);
  }

  public static void refresh() {
    pose = claclateNewPose();
  }
}
