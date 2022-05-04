// base: FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples\HardwarePushbot.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class BasicBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive;
    /*public DcMotor  leftBackDrive;
    public DcMotor  rightFrontDrive;
    public DcMotor  rightBackDrive;*/

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BasicBot_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front");
        /*leftBackDrive  = hwMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back");*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        /*leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);*/ // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        /*leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);*/

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        // Define and initialize ALL installed servos.
        //fakeServo  = hwMap.get(Servo.class, "fake_servo");

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
    }
 }

