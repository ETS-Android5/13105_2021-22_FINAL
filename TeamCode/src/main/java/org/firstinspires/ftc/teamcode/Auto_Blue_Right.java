package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;

@Autonomous(name="Auto_Blue_Right", group="Auto_Blue_Right")
public class Auto_Blue_Right extends LinearOpMode {

    Standard_Bot robot = new Standard_Bot();

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

    DistanceSensor sensorRange;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    HardwareMap hwMap = null;


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
        outtakeMotor = robot.StdOuttakeMotor;
        outtakeServo = robot.StdOuttakeServo;
        capperServo = robot.StdCapperServo;

        sensorRange = robot.StdDistanceSensor;

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double currentAngle = 0;
        double currentDistance = 0;
        double minAngle = 0;
        double minDistance = 100;
        double globalAngle = 0;
        double allianceHubLevel = 0;
        double angleToTeamElement = 0;

        waitForStart();

        while (opModeIsActive()) {

            drive(-3, -3, 0.5);             // Move away from the wall
            sleep(1000);
            rotate(25);                             // Get ready to scan
            sleep(1000);
            angleToTeamElement = rotate(-50, 0);    // Scan for the team element
            sleep(2000);
            if (angleToTeamElement < -35.0) {allianceHubLevel = 1;}
            else if (angleToTeamElement < -17.0) {allianceHubLevel = 2;}
            else {allianceHubLevel = 3;}

            // Take different actions based upon the hub level
            if (allianceHubLevel == 3) {
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setPower(-0.6);
                sleep(3000);
                intakeMotor.setPower(0.0);
                intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if (allianceHubLevel == 2){
                rotate(55);
                sleep(1000);
                outtakeArmDrive(0.3, -38, 2);
                sleep (1500);
                drive(13, 13, 0.5);
                sleep(2000);
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setPower(-0.6);
                sleep (3000);
                intakeMotor.setPower (0.0);
                intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if (allianceHubLevel == 1){

            }
            else {}     // This is an error
            outtakeArmDrive(0.3, 6, 1);
            sleep (1500);

            telemetry.update();
            break;
        }

    }
        public void drive(double right, double left, double power) {

        int rightTarget;
        int leftTarget;


        leftTarget = (int) (left + frontLeft.getCurrentPosition());
        rightTarget = (int) (right + frontRight.getCurrentPosition());

        leftTarget = leftTarget+1000;
        rightTarget = rightTarget+1000;

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);


        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double getAngle() {
        imu.getPosition();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return lastAngles.firstAngle;
    }


    public void turn(int angle){

        //resetAngle();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (angle > 0){
            frontRight.setPower(0.5);
            backRight.setPower(0.5);
            frontLeft.setPower(-0.5);
            backLeft.setPower(-0.5);
        }

        if (angle < 0) {
            frontRight.setPower(-0.5);
            backRight.setPower(-0.5);
            frontLeft.setPower(0.5);
            backLeft.setPower(0.5);
        }

        while (getAngle() > angle & opModeIsActive() & angle > 0){
            sleep(250);
            idle();
        }

        while (getAngle() < angle & opModeIsActive() & angle < 0){
            sleep(250);
            idle();
        }
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

    }
    private void rotate(int degrees)
    {
        double temp = rotate (degrees, 0);
        return;
    }

    private double rotate(int degrees, int dummy) {
        double leftPower, rightPower;
        double currentAngle = 0, currentDistance = 0, minAngle = 0, minDistance = 100;

        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = 0.5;
            rightPower = -0.5;
        } else if (degrees > 0) {
            // turn left.
            leftPower = -0.5;
            rightPower = 0.5;
        } else return 0;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && (currentAngle = getAngle()) > degrees) {
                currentDistance = sensorRange.getDistance(DistanceUnit.INCH);
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minAngle = currentAngle;
                }

            }
        } else {    // left turn.
            while (opModeIsActive() && getAngle() == 0) {
            }
            while (opModeIsActive() && (currentAngle = getAngle()) < degrees) {
            }
        }
        // turn the motors off.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // wait for rotation to stop.
        sleep(100);

        telemetry.addData("Min Dist: " + minDistance, "; Min Angle: " + minAngle);
        telemetry.update();
        // reset angle tracking on new heading.
        return minAngle;
    }
    public void outtakeArmDrive( double power, double armInches, int timeout) {
        int newTarget;

        if (opModeIsActive()) {

            newTarget = outtakeMotor.getCurrentPosition() + (int)(armInches);
            outtakeMotor.setTargetPosition(newTarget);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            outtakeMotor.setPower(power);

              while (opModeIsActive() && (runtime.seconds() < timeout) && outtakeMotor.isBusy()) {
                telemetry.addData("PathIA",  "Running to %7d :%7d", newTarget,  outtakeMotor.getCurrentPosition());
                telemetry.update();
                idle ();
            }
        }
    }
}

