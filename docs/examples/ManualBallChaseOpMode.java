package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.marrow.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.hivevisionftc.vision.BiobuzzClasses;
import org.hivevisionftc.vision.Detection;
import org.hivevisionftc.vision.FieldPose;
import org.hivevisionftc.vision.HiveVision;
import org.hivevisionftc.vision.VisionConfig;
import org.hivevisionftc.vision.weaver.WeaverRoute;

/**
 * Autonomous OpMode demonstrating continuous detection loop, obstacle avoidance,
 * and path execution using HiveVision and WeaverRoute.
 */
@Autonomous(name = "HiveVision Loop with Obstacles")
public class ManualBallChaseOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = SampleFollowerFactory.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        VisionConfig config = new VisionConfig()
                .camPitchDeg(25.0)
                .camHeight(9.2)
                .ballHeight(3.0)
                .camXOffset(4.0)
                .camYOffset(0.0)
                .minConfidence(0.40)
                .maxStalenessMs(120);

        HiveVision vision = new HiveVision(limelight, config);
        vision.setAllowedClasses(BiobuzzClasses.NECTAR_RED, BiobuzzClasses.NECTAR_BLUE);

        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            FieldPose currentPose = new FieldPose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading());

            // Fetch detections and build path avoiding an obstacle zone
            HiveVision.DetectionList list = vision.detections().fetch(currentPose);

            if (list.didDetect()) {
                Detection closest = list.getClosest();
                telemetry.addData("Closest Element", closest.getBallClass());
                telemetry.addData("Field X", closest.getX());
                telemetry.addData("Field Y", closest.getY());

                if (!follower.isBusy()) {
                    PathResult path = list.sortByDistance().with(
                            WeaverRoute.fromFetchPose()
                                    .avoid(new CircleZone(new Point(72, 72), 18.0))
                                    .maxTargets(3));

                    if (path != null) {
                        follower.follow(PedroPathingConverter.toPath(path));
                    }
                }
            } else {
                telemetry.addData("Status", "Searching for targets...");
            }

            telemetry.update();
        }
    }
}
