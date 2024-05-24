# An Intro to Setting Up Photon Vision

<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#what">What is Photon Vision</a></li>
        <li><a href="#setup">Setting Up Photon Vision</a></li>
        <li><a href="#cam">Photon Cameras</a></li>
        <li><a href="#pose">Vision Pose Estimators</a></li>
        <li><a href="#subsystem">Photon Vision Subsystem</a></li>
        <li><a href="#expand">Expanding Upon PhotonVision</a></li>

    </ol>
</details>

<div id="what"></div>

# What is Photon Vision

Photon vision is a vision processing system that uses cameras and either the rio or another coprocessor to filter the camera stream to look for vision targets. If the rio is used, then it can only use one camera and the CPU of the rio will be heavily used. 

Photon vision is really useful when multiple cameras and a coprocessor is used. Different targets can be recognized, like april tags, vision tape, or different game pieces. 

<div id="setup"></div>

# Setting Up Photon Vision

To set up photon vision, the best practice is to have atleast one camera and a coprocessor like a orange or raspberry pi. 

The first step is to install photon vision on the coprocessor of your choosing. To do this, follow the steps on the [photon vision documentation](https://docs.photonvision.org/en/latest/docs/installation/sw_install/index.html) for the chosen coprocessor. 

The next step is to install the photon vision library for wpilib. This can be found on [wpilib's website](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) and under photon vision. To install the library, follow the instructions on the website, or go to manage vendor library under the wpilib W button and intall online and paste the link.

Once that is done and the library is installed, the next step is programming

<div id="cam"></div>

# Photon Cameras

A PhotonCamera is a camera that is connected to photonvision. The first part is creating a helper class to filter information and data from each PhotonCamera. This class is called PhotonCam. 

```
   public PhotonCam(
            String camName, 
            Transform3d robotToCam,
            Swerve swerve, 
            AprilTagFieldLayout fieldLayout){
        this.fieldLayout = fieldLayout;
        cam = new PhotonCamera(camName);
        this.swerve = swerve;
        this.robotToCam = robotToCam;

        photonEstimator = 
                new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
```

The PhotonCam class takes a few parameters, these include:
* Camera Name: the name of the camera as named in the photon vision browser
* Robot to Cam: The transform3d of camera's position to the robot
* Swerve: An instance of the robot's swerve drive
* field layout: The AprilTagFieldLayout of the field that provides the pose of each april tag for the field used. 

The next part is to initialize everything. This is where you create the actual PhotonCamera object.

Additionally, a PhotonPoseEstimator is created. This will take all the robot's cameras and use their information to estimate the pose of the robot. 

```
public void update(){
    result = cam.getLatestResult();
    var visionEst = getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
            var target = result.getBestTarget();
            // robotPose is the estimated calculated pose using the field
            apriltag pose, and robot pose
            Pose3d robotPose = 
                PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(), 
                    fieldLayout.getTagPose(target.getFiducialId())
                    get(), 
                    robotToCam);

            filterAndAddVisionPos(est);
        });
}
```

The first method is the update method. This will be called by the photon vision subsystem to update each camera. The first part gets the latest result from the camera. Then it gets the global estimated position of the robot. If the vision estimation is present, then the estimation is filtered and added to the vision pose. 

There are two extra parts if the estimation is present. The best target of the result is gotten, and a different robot pose is estimated based off the camera results and the april tag pose. 

```
public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    var visionEstimation = photonEstimator.update(cam.getLatestResult());
    double latestTimestamp = cam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult){
        lastEstTimestamp = latestTimestamp;
    }
    return visionEstimation;
}
```

The getEstimatedGlobalPose method returns an optional estimated pose of the robot. It first checks to see if there is a new result based off the timestamps. After updating the PhotonPoseEstimator, the vision estimation is returned.

```
public void filterAndAddVisionPose(EstimatedRobotPose pose){
    Matrix<N3, N1> cov = VecBuilder.fill(0.4, 0.4, 0.4);

    double ambiguity = 0;
    double distance = 0;
    for (var tar : pose.targetsUsed){
            ambiguity += tar.getPoseAmbiguity() / pose.targetsUsed.size();
            distance += tar.getBestCameraToTarget().getTranslation().getNorm() / pose targetsUsed.size();
    }
    if (ambiguity > 0.25 || distance > 10){
        return;
    }

    swerve.addVisionMeasurement(pose.estimatedPose, pose.timestampSeconds, cov);
}
```

The filterAndAddVisionPose method takes the estimated robot pose, and based off of the distance and april tag ambiguity, the method will either skip and not add the measurement to the swerve vision pose estimator, or instead if the requirements are below a certian thresholds, then the measurement will be sent to the swerve vision pose estimator. 

The swerve vision pose estimator uses a matrix of standard deviations of how reliable to trust the measurements. The higher the number, the higher the trust. 

This method starts with 0.4 for each mearuement (x, y and rotation)

The method then gets the average distance and ambiguity between all the targets and if either of those are above their thresholds, then the method skips and returns void. Otherwise the measurement is add to the swerve vision pose estimation

<div id="pose"></div>

# Vision Pose Estimators

Within the swerve drive subsystem, there are needed additions to ensure photon vision works. 

```
poseEstimator = 
      new SwerveDrivePoseEstimator(
        DriveConstants.swerveKinematics,
        getHeading(),
        getModulePositions(),
        swerveOdometry.getPoseMeters());
```

The first part is to initialize a SwerveDrivePoseEstimator. This can be done by declaring it similar to the SwerveDriveOdometry, then initializing it in the constructor. 

This takes very similar parameters when compared to the SwerveDriveOdometry and they are updated with the same measurements. 

```
poseEstimator.update(getHeading(), getModulePositions());
```

Similar to odometry, the poseEstimator needs updated in periodic every time with the robot's heading and the module positions

```
/**
* Adds a vision measurement to the pose estimator
* 
* @param pose The pose measurement from vision
* @param timestamp The timestamp of the pose measurement
* @param cov The covariance matrix of the measurement
*/
public void addVisionMeasurement(Pose3d pose, double timestamp, Matrix<N3, N1> cov){
    poseEstimator.addVisionMeasurement(pose.toPose2d(), timestamp, cov);
}
```

The method add measurement is called by the PhotonCam class. This method adds the inputted pose estimation to the SwerveDrivePoseEstimator along with a timestamp and a matrix of standard deviations to determine how much to trust the measurement. 

This method is crucial as if no measurements are added, then the pose estimator acts just like odometry. 

```
public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
}
```

The final step is to create a method to access the pose from the pose estimator. This pose can be used to determine angles and measurements from the robot's pose. 

If the vision measurements are accurrate, then the estimated pose can be used for autonomous and path planner. 

<div id="subsystem"></div>

# Photon Vision Subsystem

The final step to setting up photon vision is to create the PhotonVision subsystem

At least one PhotonCam object needs declared for photon vision to work. Additional cameras can be added to view more targets. 

The second object that needs declared is a AprilTagFieldLayout called fieldLayout

```
public PhotonVision(Swerve swerve) {
    try {
      fieldLayout = 
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e){
      e.printStackTrace();
    }

    frontCamera = new PhotonCam(
      VisionConstants.frontCamName, 
      VisionConstants.frontRobotToCam, 
      swerve, fieldLayout);

    backCamera = new PhotonCam(
      VisionConstants.backCamName, 
      VisionConstants.backRobotToCam, 
      swerve, fieldLayout);
}
```

After everything is declared, everything can be initialized in the subsysyem's constructor. 

The first part is the field layout. This is put into a method that tries to initializes the field layout, because if the resource fails to load, then the robot code will break if it is not in the try method.  

The AprilTagFieldLayout is loaded from wpilib's resource library where the current year's field can be loaded. 

The second part of the constructor is initializing the cameras from the initialized vision constants. There can be as many cameras as the coprocesser allows in terms of usb ports. 

```
public void periodic() {
    frontCamera.update();
    backCamera.update();
}
```

The second main aspect of the PhotonVision subsystem is within it's periodic method. This method will run once per scheduler and update all the PhotonCams which will add the camera's vision measurements to the swerve drive subsystem. For every camera initialized, the camera should be called to update in the periodic method. 

This example has two PhotonCams so each camera calls update once. 

```
public Optional<Pose3d> getTagPose(int id){
    return fieldLayout.getTagPose(id);
}
```

There is an optional method that uses the field layout to get the pose from inputted april tag ids. 

This method can be used and called from other commands and methods without those methods needing their own AprilTagFieldLayout

The final step is initializing the PhotonVision subsystem in the RobotContainer. 

<div id="expand"></div>

# Expanding Upon PhotonVision

Once the basic methods and subsystems are created for PhotonVision, more commands can be created to use the data gathered from the vision system. 

One idea is to take the estimated robot pose and aim the robot's shooter or mechanism based off of the differences of poses between the target and the robot. There are no working commands that have been made yet, but check back to the 2024 robot's commands to see what was kinda working. 