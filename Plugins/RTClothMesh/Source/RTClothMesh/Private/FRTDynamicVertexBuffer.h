#pragma once

class FRTDynamicVertexBuffer : public FVertexBuffer
{
private:
	struct FRTVertex
	{
		FRTVertex() {}
		FRTVertex(float x, float y, float z) : X(x), Y(y), Z(z) {}
		FRTVertex(FRTVertex const& Vec) : X(Vec.X), Y(Vec.Y), Z(Vec.Z) 
		{
		}
		FRTVertex& operator=(FVector const& Vec)
		{
			X = Vec.X;
			Y = Vec.Y;
			Z = Vec.Z;
			return *this;
		}
		float X = 0;
		float Y = 0;
		float Z = 0;
	};
	int32 NumVertices = 0;
	TResourceArray<FRTVertex> VertexData;
	FUnorderedAccessViewRHIRef Position_UAV;
	FShaderResourceViewRHIRef PositionComponentSRV;
	
	FVertexBufferRHIRef CreateRHIBuffer_RenderThread()
	{
		if (NumVertices)
		{
			FRHIResourceCreateInfo CreateInfo;
			CreateInfo.ResourceArray = &VertexData;
			return RHICreateVertexBuffer(sizeof(FRTVertex) * NumVertices, BUF_UnorderedAccess | BUF_ShaderResource, CreateInfo);
		}
		return nullptr;
	}
public:
	void Init(uint32 Num)
	{
		NumVertices = Num;
		VertexData.Init({0, 0, 0}, Num);
	}

	uint32 GetNumVertices() const {return NumVertices;}

	uint32 GetDataSize() const {return NumVertices * sizeof(FRTVertex);}

	FRTVertex &VertexPosition(uint32 i)
	{
		return VertexData[i];
	}

	void BindPositionVertexBuffer(const FVertexFactory* VertexFactory, FStaticMeshDataType& StaticMeshData) const
	{
		StaticMeshData.PositionComponent = FVertexStreamComponent(
			this,
			0,
			sizeof(FRTVertex),
			VET_Float3
		);
		StaticMeshData.PositionComponentSRV = PositionComponentSRV;
	}
	
	// FRenderResource interface.
	virtual void InitRHI() override
	{
		VertexBufferRHI = CreateRHIBuffer_RenderThread();
		if (VertexBufferRHI)
		{
			Position_UAV = RHICreateUnorderedAccessView(VertexBufferRHI, PF_FloatRGB);
			PositionComponentSRV = RHICreateShaderResourceView(FShaderResourceViewInitializer(VertexBufferRHI, PF_R32_FLOAT));
		}
	}
	FUnorderedAccessViewRHIRef const& GetPositionUAV() {return this->Position_UAV;}
	virtual void ReleaseRHI() override
	{
		Position_UAV.SafeRelease();
		PositionComponentSRV.SafeRelease();
		FVertexBuffer::ReleaseRHI();
	}
	virtual FString GetFriendlyName() const override { return TEXT("PositionOnly Static-mesh vertices"); }
};