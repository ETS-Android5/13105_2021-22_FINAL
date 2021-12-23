package org.firstinspires.ftc.teamcode.RobotClasses.Subsytems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Standard_Bot {

    public DcMotorImplEx StdFrontRight   = null;
    public DcMotorImplEx StdBackRight  = null;
    public DcMotorImplEx StdFrontLeft   = null;
    public DcMotorImplEx StdBackLeft = null;
    public DcMotor StdIntakeMotor  = null;
    public DcMotor StdCapperMotor = null;
    public DcMotor StdCarouselMotor = null;
    public DcMotor StdOuttakeMotor = null;
    public Servo StdCapperServo = null;
    public Servo StdOuttakeServo = null;
    public DistanceSensor StdDistanceSensor = null;
    public Rev2mDistanceSensor StdRevDistanceSensor = null;
    public DistanceSensor StdBackStop = null;
    public Rev2mDistanceSensor StdRevBackStop = null;

    public final double Capper_Start = 0;
    public final double Outtake_Start = 0;

    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    public Standard_Bot(){

    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        StdFrontLeft = hwMap.get(DcMotorImplEx.class, "frontLeft");
        StdFrontRight = hwMap.get(DcMotorImplEx.class, "frontRight");
        StdBackLeft = hwMap.get(DcMotorImplEx.class, "backLeft");
        StdBackRight = hwMap.get(DcMotorImplEx.class, "backRight");
        StdIntakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        StdCapperMotor = hwMap.get(DcMotor.class, "capperMotor");
        StdCarouselMotor = hwMap.get(DcMotor.class, "carouselMotor");
        StdOuttakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        StdCapperServo = hwMap.get(Servo.class, "capperServo");
        StdOuttakeServo = hwMap.get(Servo.class, "outtakeServo");

        StdDistanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        StdRevDistanceSensor = (Rev2mDistanceSensor)StdDistanceSensor;
        StdBackStop = hwMap.get(DistanceSensor.class, "backStop");
        StdRevBackStop = (Rev2mDistanceSensor)StdBackStop;

        StdFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        StdFrontRight.setDirection(DcMotor.Direction.REVERSE);
        StdBackLeft.setDirection(DcMotor.Direction.FORWARD);
        StdBackRight.setDirection(DcMotor.Direction.REVERSE);
        StdIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        StdCapperMotor.setDirection(DcMotor.Direction.FORWARD);
        StdCarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        StdOuttakeMotor.setDirection(DcMotor.Direction.FORWARD);

        StdFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        StdCapperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdCarouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        StdOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        StdCapperServo.setPosition(Capper_Start);
        StdOuttakeServo.setPosition(Outtake_Start);

        // Set all motors to zero power
        StdFrontLeft.setPower(0);
        StdBackLeft.setPower(0);
        StdFrontRight.setPower(0);
        StdBackRight.setPower(0);
        StdIntakeMotor.setPower(0);
        StdCapperMotor.setPower(0);
        StdOuttakeMotor.setPower(0);
        StdCarouselMotor.setPower(0);

        // Set all motors to run without encoders.
        // May use RUN_WITHOUT_ENCODER or RUN_USING_ENCODERS

        StdFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdCapperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdOuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdCarouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

