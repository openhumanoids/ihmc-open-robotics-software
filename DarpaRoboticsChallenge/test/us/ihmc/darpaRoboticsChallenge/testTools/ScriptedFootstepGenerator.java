package us.ihmc.darpaRoboticsChallenge.testTools;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class ScriptedFootstepGenerator
{
   private final SideDependentList<ContactablePlaneBody> bipedFeet;

   public ScriptedFootstepGenerator(ReferenceFrames referenceFrames, FullRobotModel fullRobotModel)
   {
      this.bipedFeet = setupBipedFeet(referenceFrames, fullRobotModel);
   }

   public ScriptedFootstepGenerator(SideDependentList<ContactablePlaneBody> bipedFeet)
   {
      this.bipedFeet = bipedFeet;
   }

   public FootstepDataList generateFootstepsFromLocationsAndOrientations(RobotSide[] robotSides, double[][][] footstepLocationsAndOrientations)
   {
      FootstepDataList footstepDataList = new FootstepDataList();

      for (int i = 0; i < robotSides.length; i++)
      {
         RobotSide robotSide = robotSides[i];
         double[][] footstepLocationAndOrientation = footstepLocationsAndOrientations[i];
         Footstep footstep = generateFootstepFromLocationAndOrientation(robotSide, footstepLocationAndOrientation);
         FootstepData footstepData = new FootstepData(footstep);
         footstepDataList.add(footstepData);
      }

      return footstepDataList;
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[][] footstepLocationAndOrientation)
   {
      double[] location = footstepLocationAndOrientation[0];
      double[] orientation = footstepLocationAndOrientation[1];

      return generateFootstepFromLocationAndOrientation(robotSide, location, orientation);
   }

   private Footstep generateFootstepFromLocationAndOrientation(RobotSide robotSide, double[] location, double[] orientation)
   {
      ContactablePlaneBody foot = bipedFeet.get(robotSide);
      boolean trustHeight = true;
      Quat4d quaternion = new Quat4d(orientation);

      FramePoint footstepPosition = new FramePoint(ReferenceFrame.getWorldFrame(), location);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), quaternion);
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
      ReferenceFrame soleFrame = FootstepUtils.createSoleFrame(footstepPoseFrame, foot);
      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(foot, soleFrame);
      Footstep footstep = new Footstep(foot, footstepPoseFrame, soleFrame, expectedContactPoints, trustHeight);

      return footstep;
   }

   public Footstep generateFootstep(RobotSide robotSide, Point3d pointInWorldToStepTo, double footstepYaw, Vector3d surfaceNormal)
   {
      boolean trustHeight = true;
      ContactablePlaneBody foot = bipedFeet.get(robotSide);

      return generateFootstep(foot, pointInWorldToStepTo, footstepYaw, surfaceNormal, trustHeight);
   }

   private final Matrix3d tempMatrix3d = new Matrix3d();

   public Footstep generateFootstep(ContactablePlaneBody foot, Point3d pointInWorldToStepTo, double footstepHeading, Vector3d surfaceNormal,
                                    boolean trustHeight)
   {
      getRotationGivenNormalAndHeading(tempMatrix3d, surfaceNormal, footstepHeading);

      FramePoint footstepPosition = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToStepTo);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), tempMatrix3d);
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);
      ReferenceFrame soleFrame = FootstepUtils.createSoleFrame(footstepPoseFrame, foot);
      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(foot, soleFrame);
      Footstep footstep = new Footstep(foot, footstepPoseFrame, soleFrame, expectedContactPoints, trustHeight);

      return footstep;
   }

   public SideDependentList<ContactablePlaneBody> setupBipedFeet(ReferenceFrames referenceFrames, FullRobotModel fullRobotModel)
   {
      double footForward, footBack, footWidth;

      if (DRCConfigParameters.USE_GFE_ROBOT_MODEL)
      {
         footForward = DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD;
         footBack = DRCRobotParameters.DRC_ROBOT_FOOT_BACK;
         footWidth = DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH;
      }
      else
      {
         footForward = 0.17;
         footBack = 0.07;
         footWidth = 0.12;
      }

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }

   private final Vector3d tempXVector = new Vector3d();
   private final Vector3d tempYVector = new Vector3d();
   private final Vector3d tempZVector = new Vector3d();

   private void getRotationGivenNormalAndHeading(Matrix3d matrixToPack, Vector3d normal, double heading)
   {
      if (Math.abs(normal.getZ()) < 1e-7)
         throw new RuntimeException("z component of normal must not be zero!");

      // Solve for normal dotted with xVector = 0 while keeping the x and y components of xVector in the heading direction.
      double xVectorX = Math.cos(heading);
      double xVectorY = Math.sin(heading);
      double xVectorZ = -(normal.getX() * xVectorX + normal.getY() * xVectorY) / normal.getZ();

      tempXVector.set(xVectorX, xVectorY, xVectorZ);
      tempZVector.set(normal);
      tempYVector.cross(tempZVector, tempXVector);

      tempXVector.normalize();
      tempYVector.normalize();
      tempZVector.normalize();

      matrixToPack.setColumn(0, tempXVector);
      matrixToPack.setColumn(1, tempYVector);
      matrixToPack.setColumn(2, tempZVector);
   }
}
