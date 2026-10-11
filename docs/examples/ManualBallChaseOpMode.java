package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.hivevisionftc.ballchase.BallChaseWeaver;
import org.hivevisionftc.ballchase.BallTracker;

/**
 * The full BallChaseWeaver state machine when you need to keep your own loop:
 * scan, plan an obstacle-avoiding Weaver bezier, turn, chase on camera, pickup.
 *
 * <p>YOU own {@code follower.update()} - the hunt plans and drives, it never
 * spins the localization loop for you.
 */
@Autonomous(name = "Ball Chase (BallChaseWeaver loop)")
public class ManualBallChaseOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = SampleFollowerFactory.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        BallChaseWeaver hunt = new BallChaseWeaver(follower, limelight, intake);
        hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        hunt.setMaxPickups(3);
        hunt.setTimeBudgetSec(25);
        // Optional: route around a rig / static robot, field inches.
        // hunt.addObstacle(new CircleZone(new Point(72, 72), 18.0));

        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
        waitForStart();

        hunt.start();
        while (opModeIsActive() && !hunt.isDone()) {
            follower.update();
            hunt.update();
            hunt.addTelemetry(telemetry);
            telemetry.update();
        }
        hunt.abort();

        telemetry.addData("Pickups", hunt.getPickups());
        telemetry.addData("Plans", hunt.getPlanCount());
        telemetry.update();
    }
}
