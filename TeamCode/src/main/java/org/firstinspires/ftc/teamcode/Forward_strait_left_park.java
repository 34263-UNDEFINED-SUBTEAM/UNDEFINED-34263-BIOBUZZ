
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Forward_strait_left_park extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot_Pinpoint robot = new SimplifiedOdometryRobot_Pinpoint(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.
            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive(  7, 0.60, 0.25);

            telemetry.addData(">", "pause");
            telemetry.update();
            sleep(500);

            telemetry.addData(">", "Left");
            telemetry.update();
            robot.strafe(15, 0.60, 0.15);

            telemetry.addData(">", "Finished");
            telemetry.update();

        }
    }
}
