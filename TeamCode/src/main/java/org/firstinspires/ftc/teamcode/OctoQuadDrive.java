package org.firstinspires.ftc.teamcode;



import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuad;
import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuadRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
@Config
public class OctoQuadDrive extends MecanumDrive {
    public static class Params {
        /*
        Set this to the name that your Octoquad is configured as in your hardware config.
         */
        public String octoquadDeviceName = "octoquad";

        /*
        Set these to the numbers of the ports that the X and Y odometry are plugged into on the OctoQuad.
         */
        public int odometryPortX = 1;

        public int odometryPortY = 2;

        // TODO explain
        public double angularScalar = 1.0;

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        //These are tuned for 3110-0002-0001 Product Insight #1
        // RR localizer note: These units are inches, presets are converted from mm (which is why they are inexact)
        public double xOffset = -3.3071;
        public double yOffset = -6.6142;

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.

        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public OctoQuad.EncoderDirection xDirection = OctoQuad.EncoderDirection.FORWARD;
        public OctoQuad.EncoderDirection yDirection = OctoQuad.EncoderDirection.FORWARD;

        /*
        Use the OctoQuad IMU for tuning
        If true, overrides any IMU setting in MecanumDrive and uses exclusively OctoQuad for tuning
        You can also use the OctoQuad directly in MecanumDrive if this doesn't work for some reason;
         replace "imu" with "octoquad" or whatever your octoquad is called in config.
         Note: OctoQuad IMU is always used for base localization
         */
        public boolean useOctoQuadIMUForTuning = true;
    }

    public static Params PARAMS = new Params();
    public OctoQuadRR octoquad;
    private Pose2d lastOctoQuadPose = pose;

    public OctoQuadDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);
        octoquad = hardwareMap.get(OctoQuadRR.class,PARAMS.octoquadDeviceName);

        if (PARAMS.useOctoQuadIMUForTuning) {
            lazyImu = new LazyImu(hardwareMap, PARAMS.octoquadDeviceName, new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0)));
        }

        octoquad.setLocalizerPortX(PARAMS.odometryPortX);
        octoquad.setLocalizerPortY(PARAMS.odometryPortY);

        octoquad.setSingleEncoderDirection(PARAMS.odometryPortX,PARAMS.xDirection);
        octoquad.setSingleEncoderDirection(PARAMS.odometryPortY,PARAMS.yDirection);

        // RR localizer note: don't love this conversion (change driver?)
        octoquad.setLocalizerTcpOffsetMM_X((float)DistanceUnit.MM.fromInches(PARAMS.xOffset));
        octoquad.setLocalizerTcpOffsetMM_Y((float)DistanceUnit.MM.fromInches(PARAMS.yOffset));


        octoquad.setLocalizerCountsPerMM_X((float) PARAMS.encoderResolution);
        octoquad.setLocalizerCountsPerMM_Y((float) PARAMS.encoderResolution);

        octoquad.setLocalizerImuHeadingScalar((float) PARAMS.angularScalar);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        octoquad.baseInitialize();

        octoquad.writePose(pose);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOctoQuadPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override the sensor's pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            octoquad.writePose(pose);
        }
        octoquad.updatePoseVel();
        pose = octoquad.getPose();
        lastOctoQuadPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return octoquad.getVel();
    }

}
