package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.Intake
import org.firstinspires.ftc.teamcode.util.WobbleGoal
import java.lang.Math.toRadians
import java.util.*
@Autonomous
class BlueMidAuto: LinearOpMode() {
    override fun runOpMode(){
        val drive = SampleMecanumDrive(hardwareMap)
        val wobbleGoal = WobbleGoal.standard(hardwareMap)

        val intake = Intake.standard(hardwareMap)
        val shooterMotor = hardwareMap.dcMotor["shooter1"]
        shooterMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterMotor2 = hardwareMap.dcMotor["shooter2"]
        shooterMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterServo = hardwareMap.servo["pinball"]
        val flap = hardwareMap.servo["Flap"]

        // init stuff
        wobbleGoal.liftArm()
        wobbleGoal.grab()
        flap.position = 0.00 //High goal: 0.00      PS: 0.05
        shooterServo.position = 0.00

        waitForStart()

        val box = 0

        val startPose = Pose2d(-64.0, 24.0, toRadians(180.0))
        drive.poseEstimate = startPose

        val rightPowerShotTrajectory = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(Vector2d(-3.0, 12.5), toRadians(0.0))
                .build()
        val midPowerShotTrajectory = drive.trajectoryBuilder(rightPowerShotTrajectory.end(), true)
                .strafeLeft(8.0)
                .build()
        val leftPowerShotTrajectory = drive.trajectoryBuilder(midPowerShotTrajectory.end(), true)
                .strafeRight(7.0)
                .build()

        shooterMotor.power = 0.8
        shooterMotor2.power = 0.8
        flap.position = 0.025
        drive.followTrajectory(rightPowerShotTrajectory)
        sleep(500)
        shooterServo.position = 0.15
        sleep(220) //; TODO: Consider reducing
        shooterServo.position = 0.0
        drive.followTrajectory(midPowerShotTrajectory)
        shooterServo.position = 0.15
        sleep(220)
        shooterServo.position = 0.0
        drive.followTrajectory(leftPowerShotTrajectory)
        shooterServo.position = 0.15
        sleep(220)
        shooterServo.position = 0.0
        flap.position = 0.0

        lateinit var trajectory: Trajectory

        wobbleGoal.lowerArm()

        when (box){
            0 -> {
                trajectory = drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToLinearHeading(Pose2d(12.0, 41.0, toRadians(90.0)), toRadians(90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
            1 ->{
                trajectory = drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(16.0, 36.0), toRadians(90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
            2 -> {
                trajectory = drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToLinearHeading(Pose2d(60.0, 37.5, toRadians(90.0)), toRadians(90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
        }

        wobbleGoal.release()

        val backup = drive.trajectoryBuilder(trajectory.end(), false)
                .splineToLinearHeading(Pose2d(-10.0, 36.0, toRadians(0.0)), toRadians(180.0)) //pick up wobble2
                .build()
        drive.followTrajectory(backup)

        wobbleGoal.liftArm()

        val intakePath = drive.trajectoryBuilder(backup.end(), false)
                .forward(20.0,  //
                        MinVelocityConstraint(
                                Arrays.asList(
                                        AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        MecanumVelocityConstraint(0.2 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build()
        intake.togglePower()
        drive.followTrajectory(intakePath)
        intake.togglePower()

        val shoot = drive.trajectoryBuilder(intakePath.end(), false)
                .back(20.0)
                .build()
        drive.followTrajectory(shoot)

        shooterMotor.power = 1.0
        shooterMotor2.power = 1.0
        for (i in 0..2) {
            shooterServo.position = 0.15
            sleep(250)
            shooterServo.position = 0.0
            sleep(250)
        }
        shooterMotor.power = 0.0
        shooterMotor2.power = 0.0

        val park = drive.trajectoryBuilder(shoot.end(), false)
                .splineToConstantHeading(Vector2d(9.0, 36.0), toRadians(0.0))
                .build()
        drive.followTrajectory(park)
    }
}
