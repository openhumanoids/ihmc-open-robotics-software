package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootswitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ActuatorForceBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointVelocityFiniteDifferenceBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;

import java.util.ArrayList;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootswitch
{
   private final RobotQuadrant robotQuadrant;
   private final ContactablePlaneBody foot;
   private final SDFFullRobotModel fullRobotModel;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;
   private final SensorOutputMapReadOnly sensorMap;
   private final BooleanYoVariable touchdownDetected;

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, SDFFullRobotModel fullRobotModel, double totalRobotWeight,
         SensorOutputMapReadOnly sensorMap, YoVariableRegistry parentRegistry)
   {
      super(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch", parentRegistry);

      this.robotQuadrant = robotQuadrant;
      this.foot = foot;
      this.fullRobotModel = fullRobotModel;
      this.totalRobotWeight = totalRobotWeight;
      this.sensorMap = sensorMap;
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
      touchdownDetected = new BooleanYoVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected", registry);

      setupTouchdownDetectors();
   }

   protected void setupTouchdownDetectors()
   {
      ForceSensorDataHolderReadOnly forceSensorProcessedOutputs = sensorMap.getForceSensorProcessedOutputs();
      if(forceSensorProcessedOutputs.getForceSensorDefinitions().size() > 0)
      {
         ActuatorForceBasedTouchdownDetector actuatorForceBasedTouchdownDetector = new ActuatorForceBasedTouchdownDetector(robotQuadrant.toString().toLowerCase() + "_knee_pitch",
               forceSensorProcessedOutputs.getByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), 100.0, registry); //TODO magic number
         touchdownDetectors.add(actuatorForceBasedTouchdownDetector);
      }

      JointVelocityFiniteDifferenceBasedTouchdownDetector jointVelocityFiniteDifferenceBasedTouchdownDetector = new JointVelocityFiniteDifferenceBasedTouchdownDetector(
            fullRobotModel.getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), hasTouchedDown, registry);
      jointVelocityFiniteDifferenceBasedTouchdownDetector.setFootInSwingThreshold(0.05);
      jointVelocityFiniteDifferenceBasedTouchdownDetector.setTouchdownThreshold(0.1);

      touchdownDetectors.add(jointVelocityFiniteDifferenceBasedTouchdownDetector);
   }

   @Override
   public boolean hasFootHitGround()
   {
      boolean touchdown = true;
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         touchdown &= touchdownDetectors.get(i).hasTouchedDown();
      }

      touchdownDetected.set(touchdown);

      return hasTouchedDown.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2d copToPack)
   {
      copToPack.setToNaN(getMeasurementFrame());
   }

   @Override
   public void updateCoP()
   {
      yoResolvedCoP.setToZero();
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if (hasFootHitGround())
         footWrenchToPack.setLinearPartZ(totalRobotWeight / 4.0);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return foot.getSoleFrame();
   }
}
