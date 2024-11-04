package frc.robot.lib;

import java.nio.file.Path;
import java.nio.file.attribute.PosixFileAttributeView;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

public class Vision{

    // diffent Cams Orented
    public class CameraInfo {
    
        Translation3d pose;

        String name;
        String pipeLineName;

        Rotation3d rotation;

        PoseStrategy poseStrategy;
        PhotonPoseEstimator poseEstimator;
        PhotonCamera camera;

         public void resetPoseEstmator(){
            poseEstimator = new PhotonPoseEstimator(kTagLayout, poseStrategy, new Transform3d(pose, rotation));
         }

         public void resetRotation(Rotation3d rotation){
            this.rotation = rotation;
            poseEstimator.setRobotToCameraTransform(new Transform3d(pose, rotation));
         }

         public PhotonPipelineResult getLatestResiaResult(){
            return camera.getLatestResult();
         }

             private double lastEstTimestamp = 0;

             public Optional<EstimatedRobotPose> getEstmatedGlobalPose(){
                var visionEst = poseEstimator.update(getLatestResiaResult());
                double latestTimestamp = getLatestResiaResult().getTimestampSeconds();
                boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

                if (newResult) lastEstTimestamp = latestTimestamp;
                return visionEst;
            } 

    
        
    }

    /**
     * Swivle
     */
    public class Swivle extends SubsystemBase{
    
        private static List<BoundryAera> boundryAeras;
        private static Supplier<Pose2d> robotPose;

            public class SwivleInfo{
        Servo servo;
        AnalogEncoder analogEncoder;
    }

    public class BoundryAera {
        
        Translation2d bottomLeftConner;
        Translation2d topRightConner;

        List<AprilTag> aprilTags = new ArrayList<>();

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

        //object
        private CameraInfo cameraInfo;
        private SwivleInfo swivleCam;

     public Swivle(int EncoderChannle, int ServoChannle, Translation3d pose, Rotation3d rotation, String cameraName, String pipelineName, PoseStrategy poseStrategy){

        //create Swivle
        swivleCam.analogEncoder = new AnalogEncoder(EncoderChannle);
        swivleCam.servo = new Servo(ServoChannle);

        //create Camera
        cameraInfo.name = cameraName;
        cameraInfo.pipeLineName = pipelineName;
        
        cameraInfo.pose = pose;

        cameraInfo.poseStrategy = poseStrategy;

        cameraInfo.rotation = rotation;

        cameraInfo.camera = new PhotonCamera(cameraName);
        cameraInfo.poseEstimator = new PhotonPoseEstimator(kTagLayout, poseStrategy, new Transform3d(pose, rotation));
    }

    public Swivle(CameraInfo cameraInfo, SwivleInfo swivleCam){
        this.cameraInfo = cameraInfo;
        this.swivleCam = swivleCam;
    }

    @Override
    public void periodic() {
        swivleCam.servo.set(getServoInput(robotPose.get()));
        UpdatePhotonVision(getCameraRotationToRobot(robotPose.get().getRotation()));
    }

    //photon vision

    public Optional<EstimatedRobotPose> getEstmatedGlobalPoseRaw(){
        return cameraInfo.getEstmatedGlobalPose();
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

    //vision

    //static
    private static AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static AprilTagFieldLayout getFieldLayout(){
        return kTagLayout;
    }

    public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, .1);

    public static void setTagLayout(Path path){
        try{
            kTagLayout = new AprilTagFieldLayout(path);
            Swivle.refreshAllBoundryArea();
        }catch (Exception e){
            System.out.println("Could not open path ):");
        }
    }

    public static void setTagLayout(String path){
        try{
            kTagLayout = new AprilTagFieldLayout(path);
            Swivle.refreshAllBoundryArea();
        }catch (Exception e){
            System.out.println("Could not open path ):");
        }
    }

    public static void setTagLayout(List<AprilTag> apriltags, double fieldWidth, double fieldHight){
        kTagLayout = new AprilTagFieldLayout(apriltags, fieldWidth, fieldHight);
            Swivle.refreshAllBoundryArea();
    }

    // class

    List<CameraInfo> staticCaneras = new ArrayList<>();
    List<Swivle> swivleCameras = new ArrayList<>();
    calcalationMethod calcalationMethod;

    enum calcalationMethod{
        AVERAGE
    }

    public Vision(calcalationMethod CalclationMethod){
        this.calcalationMethod = CalclationMethod;
    }

    public void addStaticCamera(CameraInfo cameraInfo){
        staticCaneras.add(cameraInfo);
    }

    public void addSwivleCamera(Swivle cameraInfo){
        swivleCameras.add(cameraInfo);
    }

    public List<PhotonTrackedTarget> getAllTargets(){
        
        List<PhotonTrackedTarget> trackedTargets = new ArrayList<>();
        
        for(int i = 0; i < staticCaneras.size(); i++){
            List<PhotonTrackedTarget> trackedTargetsFor = staticCaneras.get(i).getLatestResiaResult().targets;
            for(int j = 0; j < trackedTargetsFor.size(); j++){
                if(!trackedTargets.contains(trackedTargetsFor.get(j))){
                    trackedTargets.add(trackedTargetsFor.get(j));
                }
            }
        }

        for(int i = 0; i < swivleCameras.size(); i++){
            List<PhotonTrackedTarget> trackedTargetsFor = swivleCameras.get(i).cameraInfo.getLatestResiaResult().targets;
            for(int j = 0; j < trackedTargetsFor.size(); j++){
                if(!trackedTargets.contains(trackedTargetsFor.get(j))){
                    trackedTargets.add(trackedTargetsFor.get(j));
                }
            }
        }

        return trackedTargets;
    }

    public Pose2d getPose(){

        Pose2d pose = new Pose2d();
        int poseSampleAmount = 0;

        for(int i = 0; i < swivleCameras.size(); i++){
            if(swivleCameras.get(i).cameraInfo.getEstmatedGlobalPose().isPresent()){
                Pose2d cameraPose = swivleCameras.get(i).cameraInfo.getEstmatedGlobalPose().get().estimatedPose.toPose2d();
                pose.plus(new Transform2d(cameraPose.getX(), cameraPose.getY(), cameraPose.getRotation()));
                poseSampleAmount++;
            }
        }

        for(int i = 0; i < staticCaneras.size(); i++){
            if(staticCaneras.get(i).getEstmatedGlobalPose().isPresent()){
                Pose2d cameraPose = swivleCameras.get(i).cameraInfo.getEstmatedGlobalPose().get().estimatedPose.toPose2d();
                pose.plus(new Transform2d(cameraPose.getX(), cameraPose.getY(), cameraPose.getRotation()));
                poseSampleAmount++;
            }
        }

        pose.div(poseSampleAmount);

        return pose;



    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getAllTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
          var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    
        return estStdDevs;
    }

}
