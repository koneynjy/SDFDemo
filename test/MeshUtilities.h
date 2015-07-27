#ifndef _MESHUTILITIES
#define _MESHUTILITIES
#include "Config.h"
#include "IntVector.h"
#include "kDop.h"
#include "RandomStream.h"
#include "Float16.h"
#include "GraphicMath.h"
#include "BoxSphereBounds.h"
#include "AsyncWork.h"

class FMeshBuildDataProvider
{
public:

	/** Initialization constructor. */
	FMeshBuildDataProvider(
		const TkDOPTree<const FMeshBuildDataProvider, uint32>& InkDopTree) :
		kDopTree(InkDopTree)
	{}

	// kDOP data provider interface.

	FORCEINLINE const TkDOPTree<const FMeshBuildDataProvider, uint32>& GetkDOPTree(void) const
	{
		return kDopTree;
	}

	FORCEINLINE const FMatrix& GetLocalToWorld(void) const
	{
		return FMatrix::Identity;
	}

	FORCEINLINE const FMatrix& GetWorldToLocal(void) const
	{
		return FMatrix::Identity;
	}

	FORCEINLINE FMatrix GetLocalToWorldTransposeAdjoint(void) const
	{
		return FMatrix::Identity;
	}

	FORCEINLINE float GetDeterminant(void) const
	{
		return 1.0f;
	}

private:

	const TkDOPTree<const FMeshBuildDataProvider, uint32>& kDopTree;
};

void GenerateStratifiedUniformHemisphereSamples(int32 NumThetaSteps, int32 NumPhiSteps, FRandomStream& RandomStream, TArray<FVector4>& Samples)
{
	Samples.clear();
	Samples.resize(NumThetaSteps * NumPhiSteps, FVector(0));
	for (int32 ThetaIndex = 0; ThetaIndex < NumThetaSteps; ThetaIndex++)
	{
		for (int32 PhiIndex = 0; PhiIndex < NumPhiSteps; PhiIndex++)
		{
			const float U1 = RandomStream.GetFraction();
			const float U2 = RandomStream.GetFraction();

			const float Fraction1 = (ThetaIndex + U1) / (float)NumThetaSteps;
			const float Fraction2 = (PhiIndex + U2) / (float)NumPhiSteps;

			const float R = FMath::Sqrt(1.0f - Fraction1 * Fraction1);

			const float Phi = 2.0f * (float)PI * Fraction2;
			// Convert to Cartesian
			Samples.push_back(FVector4(FMath::Cos(Phi) * R, FMath::Sin(Phi) * R, Fraction1));
		}
	}
}


class FMeshDistanceFieldAsyncTask
{
public:
	FMeshDistanceFieldAsyncTask(TkDOPTree<const FMeshBuildDataProvider, uint32>* InkDopTree,
		const TArray<FVector>* InSampleDirections,
		FBox InVolumeBounds,
		FIntVector InVolumeDimensions,
		float InVolumeMaxDistance,
		int32 InZIndex,
		TArray<FFloat16>* DistanceFieldVolume)
		:
		kDopTree(InkDopTree),
		SampleDirections(InSampleDirections),
		VolumeBounds(InVolumeBounds),
		VolumeDimensions(InVolumeDimensions),
		VolumeMaxDistance(InVolumeMaxDistance),
		ZIndex(InZIndex),
		OutDistanceFieldVolume(DistanceFieldVolume),
		bNegativeAtBorder(false)
	{}

	void DoWork();

	bool WasNegativeAtBorder() const
	{
		return bNegativeAtBorder;
	}
private:

	// Readonly inputs
	TkDOPTree<const FMeshBuildDataProvider, uint32>* kDopTree;
	const TArray<FVector>* SampleDirections;
	FBox VolumeBounds;
	FIntVector VolumeDimensions;
	float VolumeMaxDistance;
	int32 ZIndex;
	bool bNegativeAtBorder;
	// Output
	TArray<FFloat16>* OutDistanceFieldVolume;
};

void FMeshDistanceFieldAsyncTask::DoWork()
{
	FMeshBuildDataProvider kDOPDataProvider(*kDopTree);
	const FVector DistanceFieldVoxelSize(VolumeBounds.GetSize() / FVector(VolumeDimensions.X, VolumeDimensions.Y, VolumeDimensions.Z));
	const float VoxelDiameter = DistanceFieldVoxelSize.Size();

	for (int32 YIndex = 0; YIndex < VolumeDimensions.Y; YIndex++)
	{
		for (int32 XIndex = 0; XIndex < VolumeDimensions.X; XIndex++)
		{
			const FVector VoxelPosition = FVector(XIndex + .5f, YIndex + .5f, ZIndex + .5f) * DistanceFieldVoxelSize + VolumeBounds.Min;
			const int32 Index = (ZIndex * VolumeDimensions.Y * VolumeDimensions.X + YIndex * VolumeDimensions.X + XIndex);

			float MinDistance = VolumeMaxDistance;
			int32 Hit = 0;
			int32 HitBack = 0;

			for (int32 SampleIndex = 0; SampleIndex < SampleDirections->size(); SampleIndex++)
			{
				const FVector RayDirection = (*SampleDirections)[SampleIndex];

				if (FMath::LineBoxIntersection(VolumeBounds, VoxelPosition, VoxelPosition + RayDirection * VolumeMaxDistance, RayDirection))
				{
					FkHitResult Result;

					TkDOPLineCollisionCheck<const FMeshBuildDataProvider, uint32> kDOPCheck(
						VoxelPosition,
						VoxelPosition + RayDirection * VolumeMaxDistance,
						true,
						kDOPDataProvider,
						&Result);

					bool bHit = kDopTree->LineCheck(kDOPCheck);

					if (bHit)
					{
						Hit++;

						const FVector HitNormal = kDOPCheck.GetHitNormal();

						if (FVector::DotProduct(RayDirection, HitNormal) > 0
							// MaterialIndex on the build triangles was set to 1 if two-sided, or 0 if one-sided
							&& kDOPCheck.Result->Item == 0)
						{
							HitBack++;
						}

						const float CurrentDistance = VolumeMaxDistance * Result.Time;

						if (CurrentDistance < MinDistance)
						{
							MinDistance = CurrentDistance;
						}
					}
				}
			}

			const float UnsignedDistance = MinDistance;

			// Consider this voxel 'inside' an object if more than 50% of the rays hit back faces
			MinDistance *= (Hit == 0 || HitBack < SampleDirections->size() * .5f) ? 1 : -1;

			// If we are very close to a surface and nearly all of our rays hit backfaces, treat as inside
			// This is important for one sided planes
			if (UnsignedDistance < VoxelDiameter && HitBack > .95f * Hit)
			{
				MinDistance = -UnsignedDistance;
			}

			const float VolumeSpaceDistance = MinDistance / VolumeBounds.GetExtent().GetMax();

			if (MinDistance < 0 &&
				(XIndex == 0 || XIndex == VolumeDimensions.X - 1 ||
				YIndex == 0 || YIndex == VolumeDimensions.Y - 1 ||
				ZIndex == 0 || ZIndex == VolumeDimensions.Z - 1))
			{
				bNegativeAtBorder = true;
			}

			(*OutDistanceFieldVolume)[Index] = FFloat16(VolumeSpaceDistance);
		}
	}
}

#pragma region TODO


// void GenerateSignedDistanceFieldVolumeData(
// 	const FStaticMeshLODResources& LODModel,
// 	const TArray<EBlendMode>& MaterialBlendModes,
// 	const FBoxSphereBounds& Bounds,
// 	float DistanceFieldResolutionScale,
// 	bool bGenerateAsIfTwoSided,
// 	FDistanceFieldVolumeData& OutData)
// {
// 	if (DistanceFieldResolutionScale > 0)
// 	{
// 		FQueuedThreadPool ThreadPool;
// 		const FPositionVertexBuffer& PositionVertexBuffer = LODModel.PositionVertexBuffer;
// 		FIndexArrayView Indices = LODModel.IndexBuffer.GetArrayView();
// 		TArray<FkDOPBuildCollisionTriangle<uint32> > BuildTriangles;
// 
// 		FVector BoundsSize = Bounds.GetBox().GetExtent() * 2;
// 		float MaxDimension = FMath::Max(FMath::Max(BoundsSize.X, BoundsSize.Y), BoundsSize.Z);
// 
// 		// Consider the mesh a plane if it is very flat
// 		const bool bMeshWasPlane = BoundsSize.Z * 100 < MaxDimension
// 			// And it lies mostly on the origin
// 			&& Bounds.Origin.Z - Bounds.BoxExtent.Z < KINDA_SMALL_NUMBER
// 			&& Bounds.Origin.Z + Bounds.BoxExtent.Z > -KINDA_SMALL_NUMBER;
// 
// 		for (int32 i = 0; i < Indices.Num(); i += 3)
// 		{
// 			FVector V0 = PositionVertexBuffer.VertexPosition(Indices[i + 0]);
// 			FVector V1 = PositionVertexBuffer.VertexPosition(Indices[i + 1]);
// 			FVector V2 = PositionVertexBuffer.VertexPosition(Indices[i + 2]);
// 
// 			if (bMeshWasPlane)
// 			{
// 				// Flatten out the mesh into an actual plane, this will allow us to manipulate the component's Z scale at runtime without artifacts
// 				V0.Z = 0;
// 				V1.Z = 0;
// 				V2.Z = 0;
// 			}
// 
// 			const FVector LocalNormal = ((V1 - V2) ^ (V0 - V2)).GetSafeNormal();
// 
// 			// No degenerates
// 			if (LocalNormal.IsUnit())
// 			{
// 				bool bTriangleIsOpaqueOrMasked = false;
// 
// 				for (int32 SectionIndex = 0; SectionIndex < LODModel.Sections.Num(); SectionIndex++)
// 				{
// 					const FStaticMeshSection& Section = LODModel.Sections[SectionIndex];
// 
// 					if ((uint32)i >= Section.FirstIndex && (uint32)i < Section.FirstIndex + Section.NumTriangles * 3)
// 					{
// 						if (MaterialBlendModes.IsValidIndex(Section.MaterialIndex))
// 						{
// 							bTriangleIsOpaqueOrMasked = !IsTranslucentBlendMode(MaterialBlendModes[Section.MaterialIndex]);
// 						}
// 
// 						break;
// 					}
// 				}
// 
// 				if (bTriangleIsOpaqueOrMasked)
// 				{
// 					BuildTriangles.push_back(FkDOPBuildCollisionTriangle<uint32>(
// 						bGenerateAsIfTwoSided,
// 						V0,
// 						V1,
// 						V2));
// 				}
// 			}
// 		}
// 
// 		TkDOPTree<const FMeshBuildDataProvider, uint32> kDopTree;
// 		kDopTree.Build(BuildTriangles);
// 
// 		//@todo - project setting
// 		const int32 NumVoxelDistanceSamples = 1200;
// 		TArray<FVector4> SampleDirections;
// 		const int32 NumThetaSteps = FMath::TruncToInt(FMath::Sqrt(NumVoxelDistanceSamples / (2.0f * (float)PI)));
// 		const int32 NumPhiSteps = FMath::TruncToInt(NumThetaSteps * (float)PI);
// 		FRandomStream RandomStream(0);
// 		GenerateStratifiedUniformHemisphereSamples(NumThetaSteps, NumPhiSteps, RandomStream, SampleDirections);
// 		TArray<FVector4> OtherHemisphereSamples;
// 		GenerateStratifiedUniformHemisphereSamples(NumThetaSteps, NumPhiSteps, RandomStream, OtherHemisphereSamples);
// 
// 		for (int32 i = 0; i < OtherHemisphereSamples.size(); i++)
// 		{
// 			FVector4 Sample = OtherHemisphereSamples[i];
// 			Sample.Z *= -1;
// 			SampleDirections.push_back(Sample);
// 		}
// 
// 		// Meshes with explicit artist-specified scale can go higher
// 		const int32 MaxNumVoxelsOneDim = DistanceFieldResolutionScale <= 1 ? 64 : 128;
// 		const int32 MinNumVoxelsOneDim = 8;
// 
// 		//@todo - project setting
// 		const float NumVoxelsPerLocalSpaceUnit = .1f * DistanceFieldResolutionScale;
// 		FBox MeshBounds(Bounds.GetBox());
// 
// 		{
// 			const float MaxOriginalExtent = MeshBounds.GetExtent().GetMax();
// 			// Expand so that the edges of the volume are guaranteed to be outside of the mesh
// 			const FVector NewExtent(MeshBounds.GetExtent() + FVector(.2f * MaxOriginalExtent));
// 			FBox DistanceFieldVolumeBounds = FBox(MeshBounds.GetCenter() - NewExtent, MeshBounds.GetCenter() + NewExtent);
// 			const float DistanceFieldVolumeMaxDistance = DistanceFieldVolumeBounds.GetExtent().Size();
// 
// 			const FVector DesiredDimensions(DistanceFieldVolumeBounds.GetSize() * FVector(NumVoxelsPerLocalSpaceUnit));
// 
// 			const FIntVector VolumeDimensions(
// 				FMath::Clamp(FMath::TruncToInt(DesiredDimensions.X), MinNumVoxelsOneDim, MaxNumVoxelsOneDim),
// 				FMath::Clamp(FMath::TruncToInt(DesiredDimensions.Y), MinNumVoxelsOneDim, MaxNumVoxelsOneDim),
// 				FMath::Clamp(FMath::TruncToInt(DesiredDimensions.Z), MinNumVoxelsOneDim, MaxNumVoxelsOneDim));
// 
// 			OutData.Size = VolumeDimensions;
// 			OutData.LocalBoundingBox = DistanceFieldVolumeBounds;
// 			OutData.DistanceFieldVolume.AddZeroed(VolumeDimensions.X * VolumeDimensions.Y * VolumeDimensions.Z);
// 
// 			TArray<FAsyncTask<FMeshDistanceFieldAsyncTask>*> AsyncTasks;
// 
// 			for (int32 ZIndex = 0; ZIndex < VolumeDimensions.Z; ZIndex++)
// 			{
// 				FAsyncTask<FMeshDistanceFieldAsyncTask>* Task = new FAsyncTask<class FMeshDistanceFieldAsyncTask>(
// 					&kDopTree,
// 					&SampleDirections,
// 					DistanceFieldVolumeBounds,
// 					VolumeDimensions,
// 					DistanceFieldVolumeMaxDistance,
// 					ZIndex,
// 					&OutData.DistanceFieldVolume);
// 
// 				ThreadPool.AddWork(Task);
// 				//Task->StartBackgroundTask(&ThreadPool);
// 				AsyncTasks.push_back(Task);
// 			}
// 			ThreadPool.DoAllWork();
// 			bool bNegativeAtBorder = false;
// 
// 			for (int32 TaskIndex = 0; TaskIndex < AsyncTasks.size(); TaskIndex++)
// 			{
// 				FAsyncTask<FMeshDistanceFieldAsyncTask>* Task = AsyncTasks[TaskIndex];
// 				bNegativeAtBorder = bNegativeAtBorder || Task->GetTask().WasNegativeAtBorder();
// 			}
// 
// 			OutData.bMeshWasClosed = !bNegativeAtBorder;
// 			OutData.bBuiltAsIfTwoSided = bGenerateAsIfTwoSided;
// 			OutData.bMeshWasPlane = bMeshWasPlane;
// 
// 			// Toss distance field if mesh was not closed
// 			if (bNegativeAtBorder)
// 			{
// 				OutData.Size = FIntVector(0, 0, 0);
// 				OutData.DistanceFieldVolume.Empty();
// 			}
// 		}
// 	}
// }

#pragma endregion

#endif // !_MESHUTILITIES
