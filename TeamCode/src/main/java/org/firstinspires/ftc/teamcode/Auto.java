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
        capperServo = robot.StdCapperServo;

        waitForStart();

        while (opModeIsActive()) {

            drive(1, 1, 0.5);
            sleep(250);

            break;

        }
    }

    public void drive(double rightInches, double leftInches, double power) {
        int rightTarget;
        int leftTarget;

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        leftTarget = (int) (leftInches + frontLeft.getCurrentPosition());
        rightTarget = (int) (rightInches + frontRight.getCurrentPosition());

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
}

