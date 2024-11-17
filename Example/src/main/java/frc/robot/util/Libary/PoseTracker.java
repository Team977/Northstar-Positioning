package frc.robot.util.Libary;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class PoseTracker {

  private static class Updater {

    public Updater() {}

    double previusValue;
    double currentValue;

    void Update(boolean hasUpdated, double value) {
      if (hasUpdated) {
        previusValue = currentValue;
        currentValue = value;
      }
    }

    void Update(double newValue) {
      if (currentValue != newValue) {
        previusValue = currentValue;
        currentValue = newValue;
      }
    }
  }

  /** EncoderValues */
  public static interface EncoderValues {

    public default double getCurrentPosition() {
      return 0;
    }

    public default double getPreviusPosition() {
      return 0;
    }

    public default double getCurrentVelocity() {
      return 10;
    }

    public default double getPreviusVelocity() {
      return 0;
    }

    public default double getCurrentAccleration() {
      return 0;
    }

    public default double getPreviusAccleration() {
      return 0;
    }
  }

  public static interface GyroValues {

    public default Rotation2d getYaw() {
      return new Rotation2d(0);
    }
    ;

    public default Rotation2d getYawVel() {
      return new Rotation2d(0);
    }
    ;

    public default boolean Updated() {
      return true;
    }

    public default boolean AcclValuesUpdate() {
      return true;
    }

    public default Translation2d accleration() {
      return new Translation2d();
    }
  }

  public static class EncoderValuesNULL implements EncoderValues {

    public EncoderValuesNULL() {}
  }

  /** EncoderValues for Real life, using staus signals */
  public static class EncoderValuesStatusSignal implements EncoderValues {

    private StatusSignal<Double> position;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> accleration;

    private Updater positionUpdater;
    private Updater velocityUpdater;
    private Updater acclerationUpdater;

    public EncoderValuesStatusSignal(
        StatusSignal<Double> position,
        StatusSignal<Double> velocity,
        StatusSignal<Double> accleration) {
      this.position = position;
      this.velocity = velocity;
      this.accleration = accleration;
    }

    public double getCurrentPosition() {

      position.refresh();
      positionUpdater.Update(position.hasUpdated(), position.getValue());

      return positionUpdater.currentValue;
    }

    public double getPreviusPosition() {

      position.refresh();
      positionUpdater.Update(position.hasUpdated(), position.getValue());

      return positionUpdater.previusValue;
    }

    public double getCurrentVelocity() {

      velocity.refresh();
      velocityUpdater.Update(velocity.hasUpdated(), velocity.getValue());

      return velocityUpdater.currentValue;
    }

    public double getPreviusVelocity() {

      velocity.refresh();
      velocityUpdater.Update(velocity.hasUpdated(), velocity.getValue());

      return velocityUpdater.previusValue;
    }

    public double getCurrentAccleration() {

      accleration.refresh();
      acclerationUpdater.Update(accleration.hasUpdated(), accleration.getValue());

      return acclerationUpdater.currentValue;
    }

    public double getPreviusAccleration() {

      accleration.refresh();
      acclerationUpdater.Update(accleration.hasUpdated(), accleration.getValue());

      return acclerationUpdater.previusValue;
    }
  }

  /** EncoderValues for Real life, using staus signals */
  public static class EncoderValuesSIM implements EncoderValues {

    private DoubleSupplier position;
    private DoubleSupplier velocity;
    private DoubleSupplier accleration;

    private Updater positionUpdater = new Updater();
    private Updater velocityUpdater = new Updater();
    private Updater acclerationUpdater = new Updater();

    public EncoderValuesSIM(
        DoubleSupplier position, DoubleSupplier velocity, DoubleSupplier accleration) {
      this.position = position;
      this.velocity = velocity;
      this.accleration = accleration;
    }

    public double getCurrentPosition() {

      positionUpdater.Update(position.getAsDouble());

      return positionUpdater.currentValue;
    }

    public double getPreviusPosition() {

      positionUpdater.Update(position.getAsDouble());

      return positionUpdater.previusValue;
    }

    public double getCurrentVelocity() {

      velocityUpdater.Update(velocity.getAsDouble());

      return velocityUpdater.currentValue;
    }

    public double getPreviusVelocity() {

      velocityUpdater.Update(velocity.getAsDouble());

      return velocityUpdater.previusValue;
    }

    public double getCurrentAccleration() {

      acclerationUpdater.Update(accleration.getAsDouble());

      return acclerationUpdater.currentValue;
    }

    public double getPreviusAccleration() {

      acclerationUpdater.Update(accleration.getAsDouble());

      return acclerationUpdater.previusValue;
    }
  }

  public static class GyroDoubleSupplier implements GyroValues {

    private final DoubleSupplier yaw;
    private final DoubleSupplier yawVel;
    private final DoubleSupplier xAccl;
    private final DoubleSupplier yAccl;

    public GyroDoubleSupplier(
        DoubleSupplier yaw, DoubleSupplier yawVel, DoubleSupplier xAccl, DoubleSupplier yAccl) {
      this.yaw = yaw;
      this.yawVel = yawVel;
      this.xAccl = xAccl;
      this.yAccl = yAccl;
    }

    private static final double Gravity = 9.8;

    public Rotation2d getYawVel() {
      return new Rotation2d(Units.Degrees.of(yawVel.getAsDouble()));
    }
    ;

    public Rotation2d getYaw() {
      return new Rotation2d(Units.Degrees.of(yaw.getAsDouble()));
    }
    ;

    public boolean Updated() {
      return true;
    }

    public boolean AcclValuesUpdate() {
      return true;
    }

    public Translation2d accleration() {
      return new Translation2d(0, 0);
    }
  }

  public static class GyroStatusSingnal implements GyroValues {

    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVel;
    private final StatusSignal<Double> xAccl;
    private final StatusSignal<Double> yAccl;

    public GyroStatusSingnal(
        StatusSignal<Double> yaw,
        StatusSignal<Double> yawVel,
        StatusSignal<Double> xAccl,
        StatusSignal<Double> yAccl) {
      this.yaw = yaw;
      this.yawVel = yawVel;
      this.xAccl = xAccl;
      this.yAccl = yAccl;
    }

    private static final double Gravity = 9.8;

    public Rotation2d getYawVel() {
      yawVel.refresh();
      return new Rotation2d(Units.Degrees.of(yaw.getValueAsDouble()));
    }
    ;

    public Rotation2d getYaw() {
      yaw.refresh();
      return new Rotation2d(Units.Degrees.of(yaw.getValueAsDouble()));
    }
    ;

    public boolean Updated() {
      yaw.refresh();
      return yaw.hasUpdated();
    }

    public boolean AcclValuesUpdate() {
      xAccl.refresh();
      yAccl.refresh();
      return xAccl.hasUpdated() && yAccl.hasUpdated();
    }

    public Translation2d accleration() {
      xAccl.refresh();
      yAccl.refresh();
      return new Translation2d(xAccl.getValue() * Gravity, yAccl.getValue() * Gravity);
    }
  }

  /** Moudule */
  public static class Moudule {

    private static double GearRatio = 1;
    private static double whealDiameterInches = 4;

    public static double getGearRatio() {
      return GearRatio;
    }

    public static void setGearRatio(double gearRatio) {
      GearRatio = gearRatio;
    }

    public static double getWhealDiameterInches() {
      return whealDiameterInches;
    }

    public static void setWhealDiameterInches(double whealDiameterInches) {
      Moudule.whealDiameterInches = whealDiameterInches;
    }

    private final Rotation2d AbsulteEncoderOffset;

    private final EncoderValues driveEncoder;
    private final EncoderValues CANcoder;

    private final Translation2d offset;

    public Translation2d getOffset() {
      return offset;
    }

    public Moudule(
        Rotation2d absulteEncoderOffset,
        EncoderValues driveEncoder,
        EncoderValues cANcoder,
        Translation2d offset) {
      AbsulteEncoderOffset = absulteEncoderOffset;
      this.driveEncoder = driveEncoder;
      CANcoder = cANcoder;
      this.offset = offset;
    }

    public Translation2d getTransformOffset() {
      return offsetTransformLinear(
          driveEncoder.getCurrentPosition(),
          driveEncoder.getPreviusPosition(),
          CANcoder.getCurrentPosition(),
          CANcoder.getPreviusPosition());
    }

    public Translation2d getAccleration() {
      return offsetTransformCurrent(
          driveEncoder.getCurrentAccleration(), CANcoder.getPreviusPosition());
    }

    public Translation2d offsetTransformLinear(
        double CurrentDrivePosition,
        double previusDrivePosition,
        double currentCANcoderPosition,
        double previusCANcoderPosition) {

      double offsetPosition = getDistanceTravled(CurrentDrivePosition - previusDrivePosition);
      Rotation2d rotation =
          applyCANCoderOffset(new Rotation2d(Units.Rotations.of(currentCANcoderPosition)));
      return new Translation2d( // x
          offsetPosition * rotation.getCos()
          // y
          ,
          offsetPosition * rotation.getSin());
    }

    public Translation2d offsetTransformCurrent(double driveEncoder, double CANcoderRotation) {

      Rotation2d rotation =
          applyCANCoderOffset(new Rotation2d(Units.Rotations.of(CANcoderRotation)));

      return new Translation2d(
          getDistanceTravled(driveEncoder * rotation.getSin()),
          getDistanceTravled(driveEncoder * rotation.getCos()));
    }

    private Rotation2d applyCANCoderOffset(Rotation2d CANCoderValue) {
      return CANCoderValue.plus(AbsulteEncoderOffset);
    }

    private Rotation2d applyGearRatio(Rotation2d rotation) {
      return rotation.div(GearRatio);
    }

    private double getDistanceTravled(Rotation2d Rotations) {
      return applyGearRatio(Rotations).getRadians()
          * edu.wpi.first.math.util.Units.inchesToMeters(whealDiameterInches / 2);
    }

    private double getDistanceTravled(double Rotations) {
      Rotation2d rotation = new Rotation2d(Units.Rotations.of(Rotations));
      return getDistanceTravled(rotation);
    }

    public SwerveModuleState getSwerveModuleState() {

      Rotation2d rotation =
          applyCANCoderOffset(new Rotation2d(Units.Rotations.of(CANcoder.getCurrentPosition())));

      return new SwerveModuleState(getDistanceTravled(driveEncoder.getCurrentVelocity()), rotation);
    }
  }

  /** RobotTracker */
  public class OdometryTracker {

    private static List<Moudule> modules = new ArrayList<>();
    private static GyroValues gyroValues;
    public static Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));

    public static void setPose(Pose2d pose) {
      RobotTracker.pose = pose;
    }

    public static Pose2d getPose() {
      return pose;
    }

    public static void setGyroValues(GyroValues gyroValues) {
      RobotTracker.gyroValues = gyroValues;
    }

    public static void addModules(Moudule newModule) {
      modules.add(newModule);
    }

    private static StructArrayPublisher<Translation2d> swerveOffsetPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("swerve Offset Values", Translation2d.struct)
            .publish();

    public static Translation2d getOdmetryOffsetToField() {

      Translation2d[] offsetModlue = new Translation2d[modules.size()];

      Translation2d offset = new Translation2d(0, 0);

      int sampleSize = 0;
      for (int i = 0; i < modules.size(); i++) {
        sampleSize++;

        Translation2d ModulesOffset = modules.get(i).getTransformOffset();
        offsetModlue[i] = ModulesOffset;
        offset =
            new Translation2d(
                offset.getX() + ModulesOffset.getX(), offset.getY() + ModulesOffset.getY());
      }

      SmartDashboard.putNumber("sample size", sampleSize);

      offset = offset.div(sampleSize);
      offset = offset.rotateBy(gyroValues.getYaw());

      swerveOffsetPublisher.set(offsetModlue);

      return offset;
    }

    public static Translation2d getOdmetryAccl() {
      Translation2d accl = new Translation2d(0, 0);

      int sampleSize = 0;
      for (Moudule module : modules) {
        sampleSize++;
        accl.plus(module.getAccleration());
      }

      accl.div(sampleSize);
      return accl;
    }

    private static void UseAcclDataToTrustModules(double Threashould) {
      Translation2d accl = gyroValues.accleration().rotateBy(gyroValues.getYaw());
      Translation2d RobotAccl = getOdmetryAccl();

      if (!MathUtil.isNear(accl.getX(), RobotAccl.getX(), Threashould)
          || !MathUtil.isNear(accl.getY(), RobotAccl.getY(), Threashould)) {
        // module.setAccleTrust(true);
      } else {
        // module.setAccleTrust(false);
      }
    }

    private static StructArrayPublisher<SwerveModuleState> swervePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("swerve Values", SwerveModuleState.struct)
            .publish();

    private static StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("OdymetryPurePose", Pose2d.struct)
            .publish();

    private static StructPublisher<Translation2d> robotOffsetPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("robot Offset Value", Translation2d.struct)
            .publish();

    public static void updateOdymetryPose(){
      Translation2d translationOffset = getOdmetryOffsetToField();
      pose =
          pose.transformBy(
              new Transform2d(
                  translationOffset.getX(),
                  translationOffset.getY(),
                  pose.getRotation().minus(gyroValues.getYaw())));

      SwerveModuleState[] swerveModuleState = new SwerveModuleState[modules.size()];
      for (int i = 0; i < modules.size(); i++) {
        swerveModuleState[i] = modules.get(i).getSwerveModuleState();
      }

      swervePublisher.set(swerveModuleState);
      robotOffsetPublisher.set(translationOffset);
      posePublisher.set(getPose());
    }

    public static void refresh() {

      /*             if(UseAcclDataForTrustingPose && gyroValues.AcclValuesUpdate()){
          UseAcclDataToTrustModules(AcclThreashould);
      }*/

      updateOdymetryPose();
    }
  }

  public static Pose2d pose;

  public static Pose2d getPose() {
    return pose;
  }

  public static void setPose(Pose2d pose) {
    PoseTracker.pose = pose;
  }
}
