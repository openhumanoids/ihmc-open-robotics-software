package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class SpatialAccelerationCommand extends InverseDynamicsCommand<SpatialAccelerationCommand>
{
   private boolean hasWeight;
   private double weight;
   private long jacobianForNullspaceId = GeometricJacobianHolder.NULL_JACOBIAN_ID;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(SpatialAccelerationVector.SIZE);
   private RigidBody base;
   private RigidBody endEffector;

   public SpatialAccelerationCommand()
   {
      super(InverseDynamicsCommandType.TASKSPACE_MOTION);
      removeWeight();
   }

   public SpatialAccelerationCommand(long jacobianId)
   {
      this();
      setJacobianForNullspaceId(jacobianId);
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
   }

   public void setJacobianForNullspaceId(long jacobianId)
   {
      this.jacobianForNullspaceId = jacobianId;
   }

   public void set(SpatialAccelerationVector spatialAcceleration)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      resetNullspaceMultipliers();
      setSelectionMatrixToIdentity();
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      setSelectionMatrixToIdentity();
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers, DenseMatrix64F selectionMatrix)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      setSelectionMatrix(selectionMatrix);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      resetNullspaceMultipliers();

      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      resetNullspaceMultipliers();

      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   @Override
   public void set(SpatialAccelerationCommand other)
   {
      jacobianForNullspaceId = other.jacobianForNullspaceId;
      hasWeight = other.hasWeight;
      weight = other.weight;

      spatialAcceleration.set(other.getSpatialAcceleration());
      nullspaceMultipliers.set(other.getNullspaceMultipliers());
      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
   }

   public void resetNullspaceMultipliers()
   {
      nullspaceMultipliers.reshape(0, 1);
   }

   public void setNullspaceMultpliers(DenseMatrix64F nullspaceMultipliers)
   {
      this.nullspaceMultipliers.set(nullspaceMultipliers);
   }

   private void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public double getWeight()
   {
      return weight;
   }

   public long getJacobianForNullspaceId()
   {
      return jacobianForNullspaceId;
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getNullspaceMultipliers()
   {
      return nullspaceMultipliers;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public RigidBody getBase()
   {
      return base;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
      hasWeight = weight != Double.POSITIVE_INFINITY;
   }

   public void removeWeight()
   {
      setWeight(Double.POSITIVE_INFINITY);
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + "endEffector = " + endEffector.getName() + ", spatialAcceleration = " + spatialAcceleration;
      return ret;
   }
}