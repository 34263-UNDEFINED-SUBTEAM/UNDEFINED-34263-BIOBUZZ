
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GS - GARDEN STRIKE", group = "Competition")
public class GS extends RouteAutoBase {

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addLine("GS - GARDEN STRIKE READY");
        telemetry.addLine("Route 2");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        matchTimer.reset();

        // 1. Shoot current balls.
        shoot(1.5);

        // 2. Travel to garden and collect balls.
        drive(28, 0.45);
        turn(-35, 0.35);
        drive(20, 0.40);
        collect(2.5);

        // 3. Shoot collected balls.
        shoot(1.5);

        // 4. Release pollen from flower.
        pollenReleasePlaceholder();

        // 5. Move toward dropped pollen and collect.
        turn(70, 0.35);
        drive(22, 0.40);
        collect(3.0);

        stopAll();
        telemetry.addData("GS Status", "Route complete");
        telemetry.addData("Time", "%.1f sec", matchTimer.seconds());
        telemetry.update();
    }
}
