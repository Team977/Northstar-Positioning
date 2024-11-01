package frc.robot.lib;

import java.nio.file.Path;
import java.nio.file.attribute.PosixFileAttributeView;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivleCam extends SubsystemBase{

    public class CameraInfo {
    
        Translation3d pose;

        String name;
        String pipeLineName;

        Rotation3d rotation;

        PoseStrategy poseStrategy;
        PhotonPoseEstimator poseEstimator;
        PhotonCamera camera;

        
         Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
         Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, .1);
        
    }

    public class SwivleInfo{
        Servo servo;
        AnalogEncoder analogEncoder;
    }

    public class BoundryAera {
        
        Translation2d bottomLeftConner;
        Translation2d topRightConner;

        List<AprilTag> aprilTags;

        BoundryAera (Translation2d bottomLeftConner, Translation2d topRightConner, AprilTagFieldLayout aprilTagFieldLayout){

            this.bottomLeftConner = bottomLeftConner;
            this.topRightConner = topRightConner;

            refreshBoundryArea(aprilTagFieldLayout);
        }

        void refreshBoundryArea(AprilTagFieldLayout aprilTagFieldLayout){
            List<AprilTag> rawAprilTags = aprilTagFieldLayout.getTags();

            for(int i = 0; i < rawAprilTags.size(); i++){
                if(withinArea(rawAprilTags.get(i).pose.getTranslation().toTranslation2d())){
                    aprilTags.add(rawAprilTags.get(i));
                }
            }
        }
        
        boolean withinArea(Translation2d pose){
            if(pose.getX() >= bottomLeftConner.getX() && pose.getX() <= topRightConner.getX()
                    && pose.getY() >= bottomLeftConner.getY() && pose.getY() <= topRightConner.getY() ){
                return true;
            }
            else{
                return false;
            }
        }

        AprilTag findClosestAprilTag(Translation2d pose){
            
            AprilTag closestApriltag = new AprilTag(0, new Pose3d());
            double clostestApriltagDistance = 1000;

            for(int i = 0; i < aprilTags.size(); i++){
                double distance = pose.getDistance(aprilTags.get(i).pose.getTranslation().toTranslation2d());
                if(distance < clostestApriltagDistance){
                    closestApriltag = aprilTags.get(i);
                    clostestApriltagDistance = distance;
                }
            }

            return closestApriltag;
        }
    }

    //static
    private static double TimeStampTreashould = 0.01;
    private static AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    private static List<BoundryAera> boundryAeras;
    private static Supplier<Pose2d> robotPose;

    private static Rotation2d rotationRange = new Rotation2d(2 * Math.PI);

    public static void refreshAllBoundryArea(){
        for(int i = 0; i < boundryAeras.size(); i++){
            boundryAeras.get(i).refreshBoundryArea(kTagLayout);
        }
    }

    public static void setRotationRange(Rotation2d range){
        rotationRange = range;
    }

    public static void setRobotPose_Sup(Supplier<Pose2d> pose){
        robotPose = pose;
    }

    public static void addBoundryAreas(BoundryAera i_boundryAera){
        boundryAeras.add(i_boundryAera);
    }

    public static void addBoundryAreas(List<BoundryAera> i_boundryAera){
        boundryAeras = i_boundryAera;
    }

    public static List<BoundryAera> getBoundryAeras(){
        return boundryAeras;
    }

    public static AprilTagFieldLayout getFieldLayout(){
        return kTagLayout;
    }

    public static void setTagLayout(Path path){
        try{
            kTagLayout = new AprilTagFieldLayout(path);
            refreshAllBoundryArea();
        }catch (Exception e){
            System.out.println("Could not open path ):");
        }
    }

    public static void setTagLayout(String path){
        try{
            kTagLayout = new AprilTagFieldLayout(path);
            refreshAllBoundryArea();
        }catch (Exception e){
            System.out.println("Could not open path ):");
        }
    }

    public static void setTagLayout(List<AprilTag> apriltags, double fieldWidth, double fieldHight){
        kTagLayout = new AprilTagFieldLayout(apriltags, fieldWidth, fieldHight);
        refreshAllBoundryArea();
    }

    // Object Orented

    private CameraInfo cameraInfo;
    private SwivleInfo swivleCam;

    public SwivleCam(int EncoderChannle, int ServoChannle, Matrix<N3, N1> singleDev, Matrix<N3, N1> multiDev, Translation3d pose, Rotation3d rotation, String cameraName, String pipelineName, PoseStrategy poseStrategy){

        //create Swivle
        swivleCam.analogEncoder = new AnalogEncoder(EncoderChannle);
        swivleCam.servo = new Servo(ServoChannle);

        //create Camera
        cameraInfo.name = cameraName;
        cameraInfo.pipeLineName = pipelineName;
        
        cameraInfo.pose = pose;

        cameraInfo.poseStrategy = poseStrategy;

        cameraInfo.rotation = rotation;

        cameraInfo.kMultiTagStdDevs = multiDev;
        cameraInfo.kSingleTagStdDevs = singleDev;

        cameraInfo.camera = new PhotonCamera(cameraName);
        cameraInfo.poseEstimator = new PhotonPoseEstimator(kTagLayout, poseStrategy, new Transform3d(pose, rotation));
    }

    public SwivleCam(CameraInfo cameraInfo, SwivleInfo swivleCam){
        this.cameraInfo = cameraInfo;
        this.swivleCam = swivleCam;
    }

    @Override
    public void periodic() {
        swivleCam.servo.set(getServoInput(robotPose.get()));
        UpdatePhotonVision(getCameraRotationToRobot(robotPose.get().getRotation()));
    }

    //photon vision

    private double lastEstTimestamp = 0;

    public Optional<EstimatedRobotPose> getEstmatedGlobalPoseRaw(){
        var visionEst = cameraInfo.poseEstimator.update(cameraInfo.camera.getLatestResult());
        double latestTimestamp = cameraInfo.camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = cameraInfo.kSingleTagStdDevs;
        var targets = cameraInfo.camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
          var tagPose = cameraInfo.poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = cameraInfo.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    
        return estStdDevs;
      }
    

    public Rotation2d getServoRotation(){
        return new Rotation2d(Units.Rotations.of(swivleCam.analogEncoder.get()));
    }

    public Rotation2d getCameraRotationToRobot(Rotation2d RobotRotation){
        return getServoRotation().plus(RobotRotation);
    }

    private void UpdatePhotonVision(Rotation2d yaw){
        cameraInfo.poseEstimator.setRobotToCameraTransform(new Transform3d(cameraInfo.pose, new Rotation3d(cameraInfo.rotation.getX(), cameraInfo.rotation.getY(), yaw.getRadians())));
    }

    //servo tracking

    public double getServoInput(Pose2d pose){
        Rotation2d angle =  findAngle(pose);

        if(Math.abs(angle.getDegrees()) > rotationRange.getDegrees()/2){
            return 0;
        }

        return angle.getRadians() / rotationRange.getRadians() / 2 + 0.5;
    }


    public Rotation2d findAngle(Pose2d pose){

        return findRawAngle(pose.getTranslation()).plus(pose.getRotation());

    }

    private Rotation2d findRawAngle(Translation2d pose){

        Translation2d closestAprilTagPose = findClosestAprilTag(pose).pose.getTranslation().toTranslation2d();
        Translation2d offsetPosition = closestAprilTagPose.minus(pose);
        return new Rotation2d(offsetPosition.getX(),offsetPosition.getY());

    }

    private AprilTag findClosestAprilTag(Translation2d pose){
        for(int i = 0; i < boundryAeras.size(); i++){
            if(boundryAeras.get(i).withinArea(pose)){
                return boundryAeras.get(i).findClosestAprilTag(pose);
            }
        }

        return new AprilTag(0, new Pose3d());
    }





}
