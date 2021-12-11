package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;

@Autonomous(name="Auto", group="Auto")
public class Auto extends LinearOpMode {

    Standard_Bot robot = new Standard_Bot();
//    TankDrive td = new TankDrive();


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    private DcMotor carouselMotor = null;
    private DcMotor capperMotor = null;

    private Servo outtakeServo = null;
    private Servo capperServo = null;

    HardwareMap hwMap =  null;

    int encoder = 0;
    boolean dummy = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        robot.init(hardwareMap);

        frontLeft = robot.StdFrontLeft;
        frontRight = robot.StdFrontRight;
        backLeft = robot.StdBackLeft;
        backRight = robot.StdBackRight;
        intakeMotor = robot.StdIntakeMotor;
        outtakeMotor = robot.StdOuttakeMotor;
        carouselMotor = robot.StdCarouselMotor;
        capperMotor = robot.StdCapperMotor;

        capperServo = robot.StdCapperServo;
        outtakeServo = robot.StdOuttakeServo;

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            drive(0.1, 0.5);
            sleep(250);

            break;

        }
    }

    public void drive(double inches, double power) {
        int rightTarget;
        int leftTarget;

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        leftTarget = (int) (inches + frontLeft.getCurrentPosition());
        rightTarget = (int) (inches + frontRight.getCurrentPosition());

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);


        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
/*
        public void encoderDrive( double power, double leftInches, double rightInches, int timeout){

        int newLeftTarget;
        int newRightTarget;
        double directionCorrection = 0;

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * 33);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * 33);

            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            backLeft.setTargetPosition(newLeftTarget);
            backRight.setTargetPosition(newRightTarget);

            runtime.reset();                           // reset the timeout time and start motion.
            frontLeft.setPower(0.05);
            backLeft.setPower(0.05);
            frontRight.setPower(0.05);
            backRight.setPower(0.05);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            while (opModeIsActive() &&
                  (runtime.seconds() < timeout) &&
                  (frontLeft.isBusy() && frontRight.isBusy()))
            {
                // Use the gyro if you are driving straight
                if (Math.abs(leftInches - rightInches) < 0.1) {
                   frontLeft.setPower (1*(power));
                   backLeft.setPower (1*(power));
                   frontRight.setPower (1*(power));
                   backRight.setPower (1*(power));
                }

                // Display it for the driver.
                telemetry.addData("leftFront", frontLeft.getPower());
                telemetry.addData("rightFront", frontRight.getPower());
                telemetry.addData("leftRear", backLeft.getPower());
                telemetry.addData("rightRear", backRight.getPower());
                telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

    }
*/
}

