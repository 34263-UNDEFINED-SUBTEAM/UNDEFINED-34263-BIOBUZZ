
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Sample Autonomous", group = "Mr. Phil")
public class SampleAutonomous_Forward_Pinpoint extends LinearOpMode
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

            // Drive forward
            robot.drive(  84, 0.60, 0.25);
           
        }
    }
}
