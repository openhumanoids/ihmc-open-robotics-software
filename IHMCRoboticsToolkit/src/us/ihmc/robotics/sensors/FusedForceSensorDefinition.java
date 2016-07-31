package us.ihmc.robotics.sensors;

public class FusedForceSensorDefinition extends ForceSensorDefinition
{
   private final ForceSensorDefinition[] composingForceSensorDefinitions;

   public FusedForceSensorDefinition(ForceSensorDefinition forceSensorDefinition, ForceSensorDefinition[] composingForceSensorDefinitions)
   {
      super(forceSensorDefinition.getSensorName(), forceSensorDefinition.getRigidBody(), forceSensorDefinition.getTransformFromSensorToParentJoint());

      this.composingForceSensorDefinitions = composingForceSensorDefinitions;
   }

   public ForceSensorDefinition[] getComposingForceSensorDefinitions()
   {
      return composingForceSensorDefinitions;
   }
}
