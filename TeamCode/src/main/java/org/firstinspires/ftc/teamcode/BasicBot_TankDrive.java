// base: /FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples\BasicOpMode_Iterative.java

package org.firstinspires.ftc.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.TeamCode.Constants.BasicBot;

/**
 * This file is an iterative (Non-Linear) "OpMode" for TeleOp driving.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot with four motors.
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 */

@TeleOp(name="BasicBot: Tank Drive", group="Tank Drive")
public class BasicBot_TankDrive extends OpMode
{
    // Declare OpMode members.
    private BasicBot_Hardware robot = new BasicBot_Hardware();

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //BasicBot_Hardware.java includes all the hardware inits. This runs them.
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //Get the values of the joysticks in this loop. A very very large number of loops happen every second
        double leftY = gamepad1.left_stick_y;
        double rightY  =  gamepad1.right_stick_y;

        //If sticks somehow are over 1/under -1 clip to those (max motor levels)
        leftPower = Range.clip(leftY, -1.0, 1.0);
        rightPower = Range.clip(rightY, -1.0, 1.0);

        //If the sticks are within a "deadzone" defined in Constants.java (.1 range initially) set them to 0
        if(Math.abs(leftPower) <= kLeftDeadZoneY){
            leftPower = 0;
        }
        if(Math.abs(rightPower) <= kRightDeadZoneY){
            rightPower = 0;
        }

        //Send the values to the motors
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower(leftPower);
        RightFrontMotor.setPower(rightPower);
        RightBackMotor.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        RightBackMotor.setPower(0);
    }

}
