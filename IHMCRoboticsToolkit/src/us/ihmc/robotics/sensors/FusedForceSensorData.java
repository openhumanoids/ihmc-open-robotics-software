package us.ihmc.robotics.sensors;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

import java.util.List;
import java.util.Map;

public class FusedForceSensorData implements ForceSensorDataReadOnly
{
   private final DenseMatrix64F wrench = new DenseMatrix64F(Wrench.SIZE, 1);

   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   private final ForceSensorDefinition[] composingForceSensorDefinitions;
   private final Map<ForceSensorDefinition, ForceSensorData> forceSensorDataMap;

   public FusedForceSensorData(FusedForceSensorDefinition fusedForceSensorDefinition, Map<ForceSensorDefinition, ForceSensorData> forceSensorDataMap)
   {
      this.forceSensorDataMap = forceSensorDataMap;
      measurementFrame = fusedForceSensorDefinition.getSensorFrame();
      measurementLink = fusedForceSensorDefinition.getRigidBody();
      composingForceSensorDefinitions = fusedForceSensorDefinition.getComposingForceSensorDefinitions();
   }

   private final Wrench temporaryWrench = new Wrench();
   private final Wrench totalWrench = new Wrench();
   private final DenseMatrix64F tmpWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

   public void updateWrench()
   {
      wrench.zero();
      totalWrench.setToZero(measurementFrame, measurementFrame);
      for (int i = 0; i < composingForceSensorDefinitions.length; i++)
      {
         temporaryWrench.setToZero(composingForceSensorDefinitions[i].getSensorFrame(), composingForceSensorDefinitions[i].getSensorFrame());
         forceSensorDataMap.get(composingForceSensorDefinitions[i]).getWrench(temporaryWrench);

         temporaryWrench.changeFrame(measurementFrame);
         temporaryWrench.changeBodyFrameAttachedToSameBody(measurementFrame);

         totalWrench.add(temporaryWrench);
      }
      totalWrench.getMatrix(tmpWrenchMatrix);
      wrench.set(tmpWrenchMatrix);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public RigidBody getMeasurementLink()
   {
      return measurementLink;
   }

   @Override
   public void getWrench(DenseMatrix64F wrenchToPack)
   {
      wrenchToPack.set(wrench);
   }

   @Override
   public void getWrench(Wrench wrenchToPack)
   {
      wrenchToPack.changeBodyFrameAttachedToSameBody(measurementFrame);
      wrenchToPack.set(measurementFrame, wrench);
   }

   @Override public void getWrench(float[] wrenchToPack)
   {
      for(int i = 0; i < Wrench.SIZE; i++)
      {
         wrenchToPack[i] = (float) wrench.get(i, 0);
      }
   }

   public void set(FusedForceSensorData forceSensorData)
   {
      this.wrench.set(forceSensorData.wrench);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(wrench);
   }
}
