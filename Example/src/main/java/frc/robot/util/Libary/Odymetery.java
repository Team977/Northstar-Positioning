package frc.robot.util.Libary;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

public class Odymetery {

  // stores the time stamp, for the motors
  public static class TimeStamp {

    public Rotation2d position = new Rotation2d(0);
    public double time = 0;
  }

  /** gyroTimeStep */
  public static class gyroTimeStep {

    public Rotation2d rotation = new Rotation2d();
    public double xAccl = 0;
    public double yAccl = 0;
    public double time = 0;
  }

  // Module,
  // used for calclating pose offset and FOM for each Module
  public static class Module {

    private static enum CalculationMethod {
      LINEAR,
      ARK
    }

    private static final CalculationMethod calculationMethod = CalculationMethod.ARK;

    private final double GearRatio;
    private final double WhealRadius;

    public Module(double gearRatio, double WhealRadius) {
      GearRatio = gearRatio;
      this.WhealRadius = WhealRadius;
    }

    Supplier<TimeStamp> currentDriveTimeStamp;
    Supplier<TimeStamp> currentTurnTimeStamp;

    TimeStamp previusDriveTimeStamp = new TimeStamp();
    TimeStamp previusTurnTimeStamp = new TimeStamp();

    ModlueTimeStamp currentTimeStamp;

    /** ModlueTimeStamp */
    public class ModlueTimeStamp {

      double currntTimeStamp = 0;
      double previusTimeStamp = 0;

      Rotation2d rotation = new Rotation2d();

      Translation2d offset = new Translation2d();
    }

    // method used to update current Time Stamp
    public ModlueTimeStamp Update() {

      ModlueTimeStamp modlueTimeStamp = new ModlueTimeStamp();

      // set Time stamp Values so no change
      TimeStamp currentDrive = currentDriveTimeStamp.get();
      TimeStamp currentTurn = currentTurnTimeStamp.get();

      // set Avarage Values
      modlueTimeStamp.currntTimeStamp = AvarageTwoTimeStamps(currentDrive.time, currentTurn.time);
      modlueTimeStamp.previusTimeStamp =
          AvarageTwoTimeStamps(previusDriveTimeStamp.time, previusTurnTimeStamp.time);

      Rotation2d avarageTurnAngle =
          new Rotation2d(
              (currentTurn.position.getRadians() + previusTurnTimeStamp.position.getRadians()) / 2);

      Rotation2d deltaPositon = currentDrive.position.minus(previusDriveTimeStamp.position);

      Translation2d offset = new Translation2d();

      // decide uses of info based on method
      switch (calculationMethod) {
        case LINEAR:
          offset = getLinearOffset(deltaPositon, avarageTurnAngle);
          break;

        case ARK:
          Rotation2d deltaRotation = currentTurn.position.minus(previusTurnTimeStamp.position);
          offset = getArkOffset(deltaPositon, previusTurnTimeStamp.position, deltaRotation);

        default:
          break;
      }

      modlueTimeStamp.offset = offset;
      modlueTimeStamp.rotation = avarageTurnAngle;

      // set Value
      currentTimeStamp = modlueTimeStamp;

      // reset the drive values
      previusDriveTimeStamp = currentDriveTimeStamp.get();
      previusTurnTimeStamp = currentTurnTimeStamp.get();

      return modlueTimeStamp;
    }

    // uses ark method of calation
    private Translation2d getArkOffset(
        Rotation2d deltaDriveRotation, Rotation2d previusRotation, Rotation2d deltaRotation) {

      double deltaPosition = convertRotationToDistatnceMeters(deltaDriveRotation);

      // r = 180 * d position / delta Rotation * PI
      double radius = (180 * deltaPosition) / (deltaRotation.getDegrees() * Math.PI);
      if (deltaRotation.getDegrees() == 0) {
        radius = 1 / Double.MAX_VALUE;
      }

      Translation2d offset =
          new Translation2d(

              // x
              deltaRotation.getSin() * radius,

              // y
              radius - deltaRotation.getCos() * radius);

      // aling offset to Modlue Grid
      offset = offset.rotateBy(previusRotation);

      return offset;
    }

    // uses a linear mothod of callation for the offset
    private Translation2d getLinearOffset(Rotation2d deltaDriveRotation, Rotation2d AvgRotation) {
      // delta r = Gear ratio * (current drive rotation - previus drive rotation)
      // dela p = Wheal Diameter * delta r
      double deltaPosition = convertRotationToDistatnceMeters(deltaDriveRotation);

      // get the offset of the module in Linear form
      // x = cos(angle) * delta Position
      // y = sin(angle) * delta Position

      Translation2d offset =
          new Translation2d(
              // x
              AvgRotation.getCos() * deltaPosition,
              // y
              AvgRotation.getSin() * deltaPosition);

      return offset;
    }

    public Translation2d getAcclerationMetersPerScound() {
      // s = d / t
      // delta s (accleration) = delta d / delta t (change in time) (error) (diffence)
      // acceleration = delta x or y / ( t2 - t1)

      double denomator = currentTimeStamp.previusTimeStamp - currentTimeStamp.currntTimeStamp;
      double x = currentTimeStamp.offset.getX() / denomator;
      double y = currentTimeStamp.offset.getY() / denomator;

      return new Translation2d(x, y);
    }

    public static ChassisSpeeds getChassisSpeeds(ModlueTimeStamp modlueTimeStamp) {
      double TimeDiff = getTimeDiff(modlueTimeStamp);
      return new ChassisSpeeds(
          modlueTimeStamp.offset.getX() / TimeDiff,
          modlueTimeStamp.offset.getY() / TimeDiff,
          modlueTimeStamp.rotation.getRadians() / TimeDiff);
    }

    public static double getTimeDiff(ModlueTimeStamp modlueTimeStamp) {
      return modlueTimeStamp.currntTimeStamp - modlueTimeStamp.previusTimeStamp;
    }

    public Translation2d getLatestTransform() {
      return currentTimeStamp.offset;
    }

    private double AvarageTwoTimeStamps(double Current, double Previus) {
      return (Current + Previus) / 2;
    }

    private boolean checkForChange(TimeStamp previusTimeStamp, TimeStamp currentTimeStamp) {
      if (previusTimeStamp.position.getDegrees() == currentTimeStamp.position.getDegrees()) {
        return false;
      }

      return true;
    }

    private Rotation2d applyGearRatio(Rotation2d Rotation) {
      return Rotation.div(GearRatio);
    }

    private double convertRotationToDistatnceMeters(Rotation2d deltaRotation) {
      return WhealRadius * deltaRotation.getRadians();
    }

    public void setCurrentDriveTimeStamp(Supplier<TimeStamp> currentDriveTimeStamp) {
      this.currentDriveTimeStamp = currentDriveTimeStamp;
      this.previusDriveTimeStamp = currentDriveTimeStamp.get();
    }

    public void setCurrentTurnTimeStamp(Supplier<TimeStamp> currentTurnTimeStamp) {
      this.currentTurnTimeStamp = currentTurnTimeStamp;
      this.previusDriveTimeStamp = currentTurnTimeStamp.get();
    }
  }

  static List<Module> modlues = new ArrayList<>();
  static Supplier<gyroTimeStep> gyroTimeStep;

  static List<Module.ModlueTimeStamp> TimeStamps = new ArrayList<>();
  static Pose2d pose = new Pose2d();
  static Transform2d deltaPosition = new Transform2d();

  static boolean IsSIM = true;
  static SwerveDriveKinematics swerveDriveKinematics;

  private static StructPublisher<Pose2d> OdymeteryPose =
      NetworkTableInstance.getDefault()
          .getStructTopic("new Odymetry Pose", Pose2d.struct)
          .publish();

  public static void addModlue(Module module) {
    modlues.add(module);
  }

  public static void setGyroTimeStep(Supplier<gyroTimeStep> gyroTimeStep) {
    Odymetery.gyroTimeStep = gyroTimeStep;
  }

  public static Pose2d Upddate() {

    // create the using time stamps
    List<Module.ModlueTimeStamp> timeStamps = new ArrayList<>();

    Translation2d averagedOffset = new Translation2d();

    // add values
    for (Module modlue : modlues) {
      // update modlue

      Module.ModlueTimeStamp timeStamp = modlue.Update();

      timeStamps.add(timeStamp);

      averagedOffset = averagedOffset.plus(timeStamp.offset);
    }

    averagedOffset = averagedOffset.div(modlues.size());

    Rotation2d rotation = gyroTimeStep.get().rotation;

    Transform2d applyedRotation =
        new Transform2d(
            averagedOffset.getX(), averagedOffset.getY(), rotation.minus(pose.getRotation()));
    // pose.getRotation().minus(rotation));

    deltaPosition = applyedRotation;

    pose = pose.plus(applyedRotation);

    TimeStamps = timeStamps;

    OdymeteryPose.accept(pose);

    return pose;
  }

  public static double getFOM() {
    return getSkidFOM() + getAcclFOM();
  }

  private static double getSkidFOM() {
    return 0;
  }

  private static double getAcclFOM() {
    return getAcclOffset();
  }

  public static double getAcclOffset() {

    // sets gyro time stamp so no change
    gyroTimeStep timeStep = gyroTimeStep.get();

    Translation2d swerveAccl = new Translation2d(0, 0);

    // average all values
    for (Module modlue : modlues) {
      // add values
      swerveAccl = swerveAccl.plus(modlue.getAcclerationMetersPerScound());
    }

    swerveAccl = swerveAccl.div(modlues.size());
    //

    double accleDistanceOffset =
        getDistanceTraveled(new Translation2d(timeStep.xAccl, timeStep.yAccl));
    double swerveDistanceOffset = getDistanceTraveled(swerveAccl);

    double diff = Math.abs(accleDistanceOffset - swerveDistanceOffset);
    SmartDashboard.putNumber("accleration Diff", diff);

    return diff;
  }

  private static double getDistanceTraveled(Translation2d offset) {
    return Math.sqrt(Math.pow(offset.getX(), 2) + Math.pow(offset.getY(), 2));
  }

  private static Translation2d avareageModlueStates(List<Module.ModlueTimeStamp> ModluesStates) {
    Translation2d offset = new Translation2d(0, 0);

    for (Module.ModlueTimeStamp modlueTimeStamp : ModluesStates) {
      offset = offset.plus(modlueTimeStamp.offset);
    }

    offset = offset.div(ModluesStates.size());

    return offset;
  }
}
