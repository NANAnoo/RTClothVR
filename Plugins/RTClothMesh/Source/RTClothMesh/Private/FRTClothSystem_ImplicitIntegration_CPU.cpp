#include "FRTClothSystem_ImplicitIntegration_CPU.h"

DECLARE_STATS_GROUP(TEXT("RTCloth"), STATGROUP_RTCloth, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("One Frame Cost"), TIME_COST, STATGROUP_RTCloth);
DECLARE_CYCLE_STAT(TEXT("ForcesAndDerivatives"), ForcesAndDerivatives,STATGROUP_RTCloth);
DECLARE_CYCLE_STAT(TEXT("Solve Linear Equation"), SolveLinearEquation,STATGROUP_RTCloth);

void FRTClothSystem_ImplicitIntegration_CPU::TickOnce(float Duration)
{
    SCOPE_CYCLE_COUNTER(TIME_COST);
    if (!IsFirstFrame)
    {
        SCOPE_CYCLE_COUNTER(ForcesAndDerivatives)
        ForcesAndDerivatives();
    }
    IsFirstFrame = false;
    // build up equation for solver, A x = b
    // A = M -dfdx * dt * dt - dfdv * dt;
    // b = f * dt + dfdx * v * dt * dt;
    Df_Dx.Execute(A, Df_Dv,
        [this, Duration](int32 Id, float This_Value, float Other_Value)
        {
            return this->Masses[Id / 3] - (This_Value * Duration + Other_Value) * Duration;
        },
        [Duration](float This_Value, float Other_Value)
        {
            return - (This_Value * Duration + Other_Value) * Duration;
        });
    
    Df_Dx.MulVector(B.GetData(), (float *)Velocity.GetData(), Velocity.Num() * 3);
    for (int32 i = 0; i < Forces.Num(); i ++)
    {
        B[3 * i] = (B[3 * i] * Duration + Forces[i][0]) * Duration;
        B[3 * i + 1] = (B[3 * i + 1] * Duration + Forces[i][1]) * Duration;
        B[3 * i + 2] = (B[3 * i + 2] * Duration + Forces[i][2]) * Duration;
    }
    // load constraints
    TArray<uint32> ConsIds;
    Constraints.GetKeys(ConsIds);
    TArray<FRTMatrix3> ConsMats;
    ConsMats.Reserve(ConsIds.Num());
    for (auto const Id : ConsIds)
        ConsMats.Add(Constraints[Id]);
    Solver->UpdateConstraints(ConsIds, ConsMats);
    // solve equation
    TArray<float> dV;
    dV.SetNumZeroed(Velocity.Num() * 3);
    {
        SCOPE_CYCLE_COUNTER(SolveLinearEquation)
        Solver->Solve(A, B, dV);
    }
    // update position
    for (int32 i = 0; i < Velocity.Num(); i ++)
    {
        FVector const DV = {dV[3 * i], dV[3 * i + 1], dV[3 * i + 2]};
        Velocity[i] += DV;
        Mesh->Positions[i] += Duration * Velocity[i];
    }
}

void FRTClothSystem_ImplicitIntegration_CPU::ForcesAndDerivatives()
{
    for (int32 i = 0; i < Forces.Num(); i ++)
    {
        Forces[i] = Gravity * Masses[i];
    }
    for (auto &D : DampingForces)
    {
        D.Set(0, 0, 0);
    }
    Df_Dx.SetValues(0.f);
    Df_Dv.SetValues(0.f);

    if (M_Material.K_Bend > 0.01)
    {
        for (auto &Con : BendConditions)
        {
            Con.UpdateCondition(Mesh->Positions, Velocity, Mesh->TexCoords);
            Con.ComputeForces(M_Material.K_Bend, M_Material.D_Bend, Forces, Forces);
            Con.ComputeDerivatives(M_Material.K_Bend, M_Material.D_Bend, Df_Dx, Df_Dx, Df_Dv);
        }
    }
    if (M_Material.K_Shear > 0.01)
    {
        for (auto &Con : ShearConditions)
        {
            Con.UpdateCondition(Mesh->Positions, Velocity, Mesh->TexCoords);
            Con.ComputeForces(M_Material.K_Shear, M_Material.D_Shear, Forces, Forces);
            Con.ComputeDerivatives(M_Material.K_Shear, M_Material.D_Shear, Df_Dx, Df_Dx, Df_Dv);
        }
    }
    
    for (auto &Con : StretchConditions)
    {
        Con.UpdateCondition(Mesh->Positions, Velocity, Mesh->TexCoords);
        Con.ComputeForces(M_Material.K_Stretch, M_Material.D_Stretch, Forces, Forces);
        Con.ComputeDerivatives(M_Material.K_Stretch, M_Material.D_Stretch, Df_Dx, Df_Dx, Df_Dv);
    }
    
}

void FRTClothSystem_ImplicitIntegration_CPU::PrepareSimulation()
{
    IsFirstFrame = true;
    // setup shear and stretch conditions
    ShearConditions.Reserve(this->Mesh->Indices.Num() / 3);
    StretchConditions.Reserve(this->Mesh->Indices.Num() / 3);
    for (int32 i = 0; i < this->Mesh->Indices.Num() / 3; i ++)
    {
        auto &F = getFaceAt(i);
        ShearConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2]});
        StretchConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2], M_Material.Rest_U, M_Material.Rest_V});
    }
    // set up bend conditions
    uint32 PairNum = 0;
    for (auto const E : other_half_of_edge)
    {
        PairNum += (E != UNKNOWN_HALF_EDGE);
    }
    BendConditions.Reserve(PairNum / 2);

    TSet<uint32> VisitedEdges;
    VisitedEdges.Reserve(other_half_of_edge.Num());
    for (auto const Edge : other_half_of_edge)
    {
        
        if (Edge != UNKNOWN_HALF_EDGE && !VisitedEdges.Contains(Edge))
        {
            HalfEdgeRef const OtherE = otherHalfEdge(Edge);
            if (!VisitedEdges.Contains(OtherE))
            {
                uint32 const V0 = toVertexIndexOfHalfEdge(nextHalfEdge(Edge));
                uint32 const V1 = fromVertexIndexOfHalfEdge(Edge);
                uint32 const V2 = toVertexIndexOfHalfEdge(Edge);
                uint32 const V3 = toVertexIndexOfHalfEdge(nextHalfEdge(OtherE));
                BendConditions.Add({V0, V1, V2, V3, M_Material.InitTheta});
                VisitedEdges.Add(Edge);
                VisitedEdges.Add(OtherE);
            }
        }
    }

    // set up runtime simulation variables
    Velocity.SetNumZeroed(Mesh->Positions.Num());
    Forces.SetNumZeroed(Mesh->Positions.Num());
    DampingForces.SetNumZeroed(Mesh->Positions.Num());

    // set up sparse matrix
    Df_Dx.UpdateSize(Mesh->Positions.Num() * 3);
    Df_Dv.UpdateSize(Mesh->Positions.Num() * 3);
    ForcesAndDerivatives();
    Df_Dx.LockPattern();
    Df_Dv.LockPattern();

    // should have same pattern
    check(Df_Dx.HasSamePattern(Df_Dv));

    // set up layout of A
    A = FRTBBSSMatrix<float>::MatrixFromOtherPattern(Df_Dx);

    // alloc memory for B
    B.SetNumUninitialized(Mesh->Positions.Num() * 3);

    // prepare solver
    Solver->Init(A);
}