package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Gyro;

@Autonomous(name="Delay_Blue_Right", group="Delay_Blue_Right")
public class Delay_Blue_Right extends LinearOpMode {

    Standard_Bot robot = new Standard_Bot();
    TankDrive drivetrain = new TankDrive();
    Gyro gyro = new Gyro();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorImplEx frontLeft = null;
    private DcMotorImplEx backLeft = null;
    private DcMotorImplEx frontRight = null;
    private DcMotorImplEx backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    private DcMotor carouselMotor = null;
    private DcMotor capperMotor = null;

    private Servo outtakeServo = null;
    private Servo capperServo = null;

    DistanceSensor sensorRange;
    DistanceSensor backStop;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    HardwareMap hwMap = null;


    @Override
    public void runOpMode() {

        telemetry.addData("Ready", "");

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
        backStop = robot.StdBackStop;

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double allianceHubLevel = 0;
        double angleToTeamElement = 0;

        telemetry.addData("ready", "");

        frontLeft.setVelocity(0, AngleUnit.DEGREES);
        frontRight.setVelocity(0, AngleUnit.DEGREES);
        backLeft.setVelocity(0, AngleUnit.DEGREES);
        backRight.setVelocity(0, AngleUnit.DEGREES);
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        capperMotor.setPower(0);
        carouselMotor.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            sleep(6000);
            drive(-5, -5, 360); // Move away from the wall
            sleep(250);
            rotate(-30); // Get ready to scan
            sleep(250);
            angleToTeamElement = rotate(18, 0); // Scan for the team element
            sleep(250);
            if (angleToTeamElement < -10) {allianceHubLevel = 3;}
            else if (angleToTeamElement > -10 && angleToTeamElement < 10) {allianceHubLevel = 2;}
            else {allianceHubLevel = 1;}
            telemetry.addData("angleToTeamElement", String.valueOf(angleToTeamElement));
            telemetry.update();
            sleep(250);

            if (allianceHubLevel == 3) {
                drive(-23, -23,360);
                sleep(250);
                threeDump();
                drive(11, 11, 360);
                sleep(250);
                carousel();
                sleep(250);
            }
            else if (allianceHubLevel == 2) {
                drive(-15, -15, 360);
                sleep(250);
                twoDump();
                drive(3,3,360);
                sleep(250);
                carousel();
                sleep(250);
            }
            else if (allianceHubLevel == 1) {
                drive(-22, -22, 360);
                sleep(250);
                oneDump();
                drive(10, 10, 360);
                sleep(250);
                carousel();
                sleep(250);
            }

            sleep(1500);

            telemetry.update();
            break;
        }

    }
    public void drive(double right, double left, double anglrt) {

        int rightTarget;
        int leftTarget;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTarget = (int) (left * (33) + frontLeft.getCurrentPosition());
        rightTarget = (int) (right * (33) + frontRight.getCurrentPosition());

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        frontLeft.setVelocity(90, AngleUnit.DEGREES);
        backLeft.setVelocity(90, AngleUnit.DEGREES);
        frontRight.setVelocity(90, AngleUnit.DEGREES);
        backRight.setVelocity(90, AngleUnit.DEGREES);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        backLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        frontRight.setVelocity(anglrt, AngleUnit.DEGREES);
        backRight.setVelocity(anglrt, AngleUnit.DEGREES);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if (backStop.getDistance(DistanceUnit.INCH) < 6 ){
                frontLeft.setVelocity(0);
                frontRight.setVelocity(0);
                backLeft.setVelocity(0);
                backRight.setVelocity(0);
                break;
            }
            idle();
        }

    /*  frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        frontLeft.setVelocity(0, AngleUnit.DEGREES);
        frontRight.setVelocity(0, AngleUnit.DEGREES);
        backLeft.setVelocity(0, AngleUnit.DEGREES);
        backRight.setVelocity(0, AngleUnit.DEGREES);

    }

    public double getAngle() {
        imu.getPosition();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return lastAngles.firstAngle;
    }

    private void rotate(int degrees) {
        double temp = rotate(degrees, 0);
        return;
    }

    private double rotate(int degrees, int dummy) {
        double leftPower, rightPower;
        double currentAngle = 0, currentDistance = 0, minAngle = 0, minDistance = 100;

        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = 270;
            rightPower = -270;
        } else if (degrees > 0) {
            // turn left.
            leftPower = -270;
            rightPower = 270;
        } else return 0;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        frontLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        backLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        frontRight.setVelocity(rightPower, AngleUnit.DEGREES);
        backRight.setVelocity(rightPower, AngleUnit.DEGREES);

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
                currentDistance = sensorRange.getDistance(DistanceUnit.INCH);
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minAngle = currentAngle;
                }
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

    public void outtakeArmDrive(double power, double armInches, int timeout) {
        int newTarget;

        if (opModeIsActive()) {

            newTarget = outtakeMotor.getCurrentPosition() + (int) (armInches);
            outtakeMotor.setTargetPosition(newTarget);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            outtakeMotor.setPower(power);

            while (opModeIsActive() && (runtime.seconds() < timeout) && outtakeMotor.isBusy()) {
                telemetry.addData("PathIA", "Running to %7d :%7d", newTarget, outtakeMotor.getCurrentPosition());
                telemetry.update();
                idle();
                outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void threeDump() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeServo.setPosition(0.15);
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeServo.setPosition(0);
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeMotor.setPower(0.5);//down
        sleep(500);
        outtakeServo.setPosition(0.15);
        sleep(100);
        outtakeMotor.setPower(0.5);
        sleep(400);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0);
    }

    public void twoDump() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeServo.setPosition(0.15);
        sleep(250);
        outtakeMotor.setPower(0.5);
        sleep(500);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0.5);
        sleep(500);
        outtakeServo.setPosition(0.15);//down
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0);
        sleep(250);
    }

    public void oneDump() {
        outtakeServo.setPosition(0.55);
        sleep(500);
        outtakeServo.setPosition(0);
        sleep(250);
        outtakeServo.setPosition(0);
        sleep(250);
    }

    public void carousel() {
        rotate(-80);
        drive(-50, -50, 360);
        sleep(250);
        drive(5, 5, 360);
        sleep(250);
        rotate(-170);
        drive(-11, -11, 180);
        capperMotor.setPower(-0.5);
        sleep(800);
        capperMotor.setPower(0);
        sleep(250);
        carouselMotor.setPower(-0.3);
        sleep(5000);
        carouselMotor.setPower(0);
        sleep(250);
        drive(19, 19, 360);
    }
}