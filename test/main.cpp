#include "kDop.h"
#include "MeshUtilities.h"


int main(){
	TkDOPTree<const FMeshBuildDataProvider, uint32> kDopTree;
	std::vector<FkDOPBuildCollisionTriangle<uint32>> vec;
	FVector4 v0, v1, v2;
	for (uint32 i = 0; i < 10000; i++){
		vec.push_back(FkDOPBuildCollisionTriangle<uint32>(i, v0, v1, v2));
	}
	kDopTree.Build(vec);

}