package us.ihmc.robotics.sensors;

import us.ihmc.robotics.screwTheory.Wrench;

import java.util.List;

public interface ForceSensorDataHolderReadOnly
{
   public abstract ForceSensorDataReadOnly get(ForceSensorDefinition forceSensor);

   public abstract ForceSensorDataReadOnly getFusedSensorData(ForceSensorDefinition fusedForceSensor);

   public abstract ForceSensorDataReadOnly getByName(String name);

   public abstract List<ForceSensorDefinition> getForceSensorDefinitions();

   public abstract List<FusedForceSensorDefinition> getFusedForceSensorDefinitions();

   public abstract void getForceSensorValue(ForceSensorDefinition key, Wrench wrenchToPack);

   public abstract ForceSensorDefinition findForceSensorDefinition(String name);
}
