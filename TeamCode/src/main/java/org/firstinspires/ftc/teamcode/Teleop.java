package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;

@TeleOp (name="Teleop", group="Teleop")
public class Teleop extends LinearOpMode {
// this is a change

    TankDrive tankDrive = new TankDrive();
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

    private GyroSensor boxGyro = null;

    HardwareMap hwMap =  null;

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

        boxGyro = robot.StdGyroSensor;

        double xValue = 0;
        double yValue = 0;
        double zValue = 0;

        waitForStart();
        runtime.reset();

        while (boxGyro.isCalibrating()) {
        }

        while (opModeIsActive()) {

            intakeMotor.setPower(0);
            outtakeMotor.setPower(0);
            carouselMotor.setPower(0);
            capperMotor.setPower(0);

            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            // Flip these signs if the robot rotates the wrong way
            frontLeft.setPower(drivePower + rotatePower);
            frontRight.setPower(drivePower - rotatePower);
            backLeft.setPower(drivePower + rotatePower);
            backRight.setPower(drivePower - rotatePower);

            while (gamepad2.a){
                intakeMotor.setPower(0.5);
            }
            while (gamepad2.b){
                intakeMotor.setPower(-0.5);
            }
            while (gamepad2.y){
                xValue = boxGyro.rawX();
                yValue = boxGyro.rawY();
                zValue = boxGyro.rawZ();

                outtakeMotor.setPower(0.1);


            }
            while (gamepad2.x){
                outtakeMotor.setPower(-0.25);
            }
            while (gamepad2.left_bumper){
                capperMotor.setPower(0.25);
            }
            while (gamepad2.right_bumper){
                capperMotor.setPower(-0.25);
            }
            while (gamepad1.a){
                carouselMotor.setPower(0.5);
            }
            while (gamepad1.b){
                carouselMotor.setPower(-0.5);
            }
            if (gamepad2.dpad_up){
                outtakeServo.setPosition(95);
            }
            if (gamepad2.dpad_down){
                outtakeServo.setPosition(0);
            }
            if (gamepad2.dpad_left){
                capperServo.setPosition(180);
            }
            if (gamepad2.dpad_right) {
                capperServo.setPosition(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gyro Readings", String.valueOf(xValue), "x", String.valueOf(yValue), "y", String.valueOf(zValue), "z");
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
