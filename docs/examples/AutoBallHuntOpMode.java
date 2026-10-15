package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.marrow.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.hivevisionftc.vision.BiobuzzClasses;
import org.hivevisionftc.vision.FieldPose;
import org.hivevisionftc.vision.HiveVision;
import org.hivevisionftc.vision.VisionConfig;
import org.hivevisionftc.vision.weaver.WeaverRoute;

/**
 * Autonomous OpMode demonstrating target detection and Weaver route generation using HiveVision.
 */
@Autonomous(name = "HiveVision Weaver Auto")
public class AutoBallHuntOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = SampleFollowerFactory.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Define vision configuration parameters
        VisionConfig config = new VisionConfig()
                .camPitchDeg(25.0)
                .camHeight(9.2)
                .ballHeight(3.0)
                .camXOffset(4.0)
                .camYOffset(0.0)
                .minConfidence(0.44)
                .maxStalenessMs(120);

        // Initialize HiveVision session
        HiveVision vision = new HiveVision(limelight, config);
        vision.setAllowedClasses(BiobuzzClasses.NECTAR_RED, BiobuzzClasses.NECTAR_BLUE);

        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Obtain current robot pose
            FieldPose robotPose = new FieldPose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading());

            // Fetch detections and generate a Weaver path route
            PathResult pathResult = vision.detections()
                    .fetch(robotPose)
                    .sortByDistance()
                    .with(WeaverRoute.fromFetchPose().maxTargets(2));

            if (pathResult != null) {
                follower.follow(PedroPathingConverter.toPath(pathResult));
                while (opModeIsActive() && follower.isBusy()) {
                    follower.update();
                    telemetry.addData("Status", "Following generated route");
                    telemetry.update();
                }
            } else {
                telemetry.addData("Status", "No targets detected");
                telemetry.update();
            }
        }
    }
}
