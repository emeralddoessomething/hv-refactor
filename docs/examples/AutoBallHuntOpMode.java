package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.hivevisionftc.ballchase.BallHunt;

/**
 * The one-line autonomous: pick up two red balls within 25 seconds, obstacle-free.
 *
 * <p>Hardware names used: "limelight" (Limelight3A) and "intake" (DcMotor) -
 * match them to the Driver Station config.
 */
@Autonomous(name = "Ball Hunt (BallHunt one-liner)")
public class AutoBallHuntOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = SampleFollowerFactory.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
        waitForStart();

        // go() owns follower.update() and the telemetry loop, and aborts cleanly.
        int picked = new BallHunt(follower, limelight, intake)
                .reds()
                .collect(2)
                .within(25)
                .go(this);

        telemetry.addData("Picked up", picked);
        telemetry.update();
        // ...then go score...
    }
}
