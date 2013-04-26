package us.ihmc.sensorProcessing.stateEstimation;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.MeasurementModelElement;
import us.ihmc.utilities.math.MatrixTools;

public class MeasurementModel
{
   private final List<MeasurementModelElement> measurementModelElements;

   private final Map<MeasurementModelElement, Integer> measurementStartIndices = new LinkedHashMap<MeasurementModelElement, Integer>();
   private final Map<ControlFlowOutputPort<?>, Integer> stateStartIndices;

   private final DenseMatrix64F H = new DenseMatrix64F(1, 1); // reshape dynamically
   private final DenseMatrix64F R = new DenseMatrix64F(1, 1); // reshape dynamically
   private final DenseMatrix64F residual = new DenseMatrix64F(1, 1); // reshape dynamically

   private final Map<MeasurementModelElement,Integer> measurementSizes = new LinkedHashMap<MeasurementModelElement, Integer>();
   private final int stateMatrixSize;
   private int measurementOutputMatrixSize;

   public MeasurementModel(List<MeasurementModelElement> measurementModelElements, Map<ControlFlowOutputPort<?>, Integer> stateStartIndices, int stateMatrixSize)
   {
      this.measurementModelElements = measurementModelElements;
      this.stateStartIndices = stateStartIndices;
      this.stateMatrixSize = stateMatrixSize;
      reshapeMatrices(); // TODO: get rid of this
   }

   public void updateMatrices()
   {
      reshapeMatrices();

      // TODO: check if necessary:
      H.zero();
      R.zero();

      for (MeasurementModelElement measurementModelElement : measurementModelElements)
      {
         measurementModelElement.computeMatrixBlocks();
         int measurementStartIndex = measurementStartIndices.get(measurementModelElement);

         for (ControlFlowOutputPort<?> statePort : measurementModelElement.getStatePorts())
         {
            int stateStartIndex = stateStartIndices.get(statePort);
            DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
            CommonOps.insert(outputMatrixBlock, H, measurementStartIndex, stateStartIndex);
         }
         DenseMatrix64F measurementCovarianceBlock = measurementModelElement.getMeasurementCovarianceMatrixBlock();
         CommonOps.insert(measurementCovarianceBlock, R, measurementStartIndex, measurementStartIndex);
      }
   }

   public DenseMatrix64F computeResidual()
   {
      for (MeasurementModelElement measurementModelElement : measurementModelElements)
      {
         DenseMatrix64F residualBlock = measurementModelElement.computeResidual();
         int measurementStartIndex = measurementStartIndices.get(measurementModelElement);
         CommonOps.insert(residualBlock, residual, measurementStartIndex, 0);
      }
      return residual;
   }

   public DenseMatrix64F getOutputMatrix()
   {
      return H;
   }

   public DenseMatrix64F getMeasurementCovarianceMatrix()
   {
      return R;
   }

   private static int computeTotalSize(Iterable<Integer> sizes)
   {
      int ret = 0;
      for (Integer size : sizes)
      {
         ret += size;
      }
      return ret;
   }

   private static void computeMeasurementSizes(List<MeasurementModelElement> measurementModelElements, Map<MeasurementModelElement, Integer> measurementSizes)
   {
      for (MeasurementModelElement measurementModelElement : measurementModelElements)
      {
         Set<ControlFlowOutputPort<?>> measurementModelElementStates = measurementModelElement.getStatePorts();
         for (ControlFlowOutputPort<?> statePort : measurementModelElementStates)
         {
            DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
            checkOrAddSize(measurementSizes, measurementModelElement, outputMatrixBlock.getNumRows());
         }
      }
   }

   private static <KeyType> void checkOrAddSize(Map<KeyType, Integer> sizes, KeyType key, int newSize)
   {
      Integer stateSize = sizes.get(key);
      if (stateSize == null)
      {
         sizes.put(key, newSize);
      }
      else if (stateSize != newSize)
      {
         throw new RuntimeException("State matrix size is ambiguous");
      }
   }

   public int getOutputMatrixSize()
   {
      return measurementOutputMatrixSize;
   }

   private void reshapeMatrices()
   {
      measurementSizes.clear();
      computeMeasurementSizes(measurementModelElements, measurementSizes);
      MatrixTools.computeIndicesIntoVector(measurementModelElements, measurementStartIndices, measurementSizes);
      measurementOutputMatrixSize = computeTotalSize(measurementSizes.values());
      H.reshape(measurementOutputMatrixSize, stateMatrixSize);
      R.reshape(measurementOutputMatrixSize, measurementOutputMatrixSize);
      residual.reshape(measurementOutputMatrixSize, 1);
   }
}
