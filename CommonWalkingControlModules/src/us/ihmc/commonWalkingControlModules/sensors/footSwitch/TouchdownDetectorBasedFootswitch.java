package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

import java.util.ArrayList;

public abstract class TouchdownDetectorBasedFootswitch implements FootSwitchInterface
{
   protected final YoVariableRegistry registry;
   protected final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   protected final BooleanYoVariable hasTouchedDown;

   public TouchdownDetectorBasedFootswitch(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = parentRegistry;
      hasTouchedDown = new BooleanYoVariable(name + "_hasTouchedDown", parentRegistry);
   }

   @Override
   public void reset()
   {
      hasTouchedDown.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      hasTouchedDown.set(hasFootHitGround);
   }
}
