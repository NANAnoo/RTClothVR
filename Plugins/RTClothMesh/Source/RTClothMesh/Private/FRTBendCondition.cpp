#include "FRTBendCondition.h"

void FRTBendCondition::UpdateCondition(
		TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV
	)
{
	auto const & X0 = X[V_idx[0]];
	auto const & X1 = X[V_idx[1]];
	auto const & X2 = X[V_idx[2]];
	auto const & X3 = X[V_idx[3]];

	auto const & V0 = V[V_idx[0]];
	auto const & V1 = V[V_idx[1]];
	auto const & V2 = V[V_idx[2]];
	auto const & V3 = V[V_idx[3]];

	L = (UV[V_idx[1]] - UV[V_idx[2]]).Size();
	
	// get edge vectors
	E01 = X1 - X0;
	E02 = X2 - X0;
	E32 = X2 - X3;
	E31 = X1 - X3;
	E21 = X1 - X2;

	// get normalized edge vectors
	auto const NE01 = E01 / E01.Size();
	auto const NE02 = E02 / E02.Size();
	auto const NE32 = E32 / E32.Size();
	auto const NE31 = E31 / E31.Size();
	auto const NE21 = E21 / E21.Size();

	// Get Cosine Value at each vertex
	// auto const c00 = NE01 | NE02;
	c01 = NE01 | NE21;
	c02 = - (NE02| NE21);

	// auto const c13 = NE31 | NE32;
	c11 = NE21 | NE31;
	c12 = -(NE32| NE21);

	// normalized triangle normals:
	n0 = (NE21 ^ NE01); n0.Normalize();
	n1 = (NE32 ^ NE21); n1.Normalize();

	// normalized bi-normals:
	b00 = (NE01 - NE21 * (NE21|NE01));b00.Normalize();
	b01 = (NE02 * (NE02|NE01)-NE01);b01.Normalize();
	b02 = (NE01 * (NE01|NE02)-NE02);b02.Normalize();

	b13 = (NE32 - NE21 * (NE21|NE32));b13.Normalize();
	b12 = (NE31 * (NE31|NE32) - NE32);b12.Normalize();
	b11 = (NE32 * (NE32|NE31) - NE31);b11.Normalize();

	// triangle heights at each vertex
	d00 = b00 | E01;
	d01 = -(b01 | E21);
	d02 = -(b02 | E02);

	d11 = - (b11 | E31);
	d12 = b12 | E21;
	d13 = b13 | E31;

	// angle between triangles
	Theta = atan2(n1 | b00, n0 | n1);

	// derivatives of theta with respect to the different vertex positions:
	dTheta_dX0 = -n0 / d00;
	dTheta_dX1 = c02 * n0 / d01 + c12 * n1 / d11;
	dTheta_dX2 = c01 * n0 / d02 + c11 * n1 / d12;
	dTheta_dX3 = -n1 / d13;

	//  derivatives of theta with time
	dTheta_dt = (dTheta_dX0 | V0) + (dTheta_dX1 | V1) + (dTheta_dX2 | V2) + (dTheta_dX3 | V3);
}
	
void FRTBendCondition:: ComputeForces(
	 float K, float D,
	TArray<FVector> &Forces, TArray<FVector> &DampingF
) {
	const float KL = K * L;
	Forces[V_idx[0]] -= KL * Theta * dTheta_dX0;
	Forces[V_idx[1]] -= KL * Theta * dTheta_dX1;
	Forces[V_idx[2]] -= KL * Theta * dTheta_dX2;
	Forces[V_idx[3]] -= KL * Theta * dTheta_dX3;

	const float DL = D * L;
	DampingF[V_idx[0]] -= DL * dTheta_dt * dTheta_dX0;
	DampingF[V_idx[1]] -= DL * dTheta_dt * dTheta_dX1;
	DampingF[V_idx[2]] -= DL * dTheta_dt * dTheta_dX2;
	DampingF[V_idx[3]] -= DL * dTheta_dt * dTheta_dX3;
}
	
void FRTBendCondition:: ComputeDerivatives(
	float K, float D,
	FRTBBSSMatrix<float> &dfdx,
	FRTBBSSMatrix<float> &dddx,
	FRTBBSSMatrix<float> &dddv
)
{
	// derivatives of normal with respect to the different vertex positions:
	auto const dn0dP0 = FRTMatrix3::CrossVec(b00, n0) / d00;
	auto const dn0dP1 =  FRTMatrix3::CrossVec(b01, n0) / d01;
	auto const dn0dP2 =  FRTMatrix3::CrossVec(b02, n0) / d02;
	auto const dn0dP3 = FRTMatrix3::Zero();

	auto const dn1dP0 = FRTMatrix3::Zero();
	auto const dn1dP1 =  FRTMatrix3::CrossVec(b11, n1) / d11;
	auto const dn1dP2 =  FRTMatrix3::CrossVec(b12, n1) / d12;
	auto const dn1dP3 =  FRTMatrix3::CrossVec(b13, n1) / d13;

	// derivatives of the cosines:
	auto const dc01dP0 = -b02 * ((b00|E01) / (E01|E01));
	auto const dc01dP2 = -b00 * ((b02|E21) / (E21|E21));
	auto const dc01dP1 = -dc01dP0 - dc01dP2;
	auto const dc01dP3 = FVector::ZeroVector;

	auto const dc02dP0 = -b01 * ((b00|E02) / (E02|E02));
	auto const dc02dP1 = b00 * ((b01|E21) / (E21|E21));
	auto const dc02dP2 = -dc02dP0 - dc02dP1;
	auto const dc02dP3 = FVector::ZeroVector;

	auto const dc11dP0 = FVector::ZeroVector;
	auto const dc11dP2 = -b13 * ((b12|E21) / (E21|E21));
	auto const dc11dP3 = -b12 * ((b13|E31) / (E31|E31));
	auto const dc11dP1 = -dc11dP2 - dc11dP3;

	auto const dc12dP0 = FVector::ZeroVector;
	auto const dc12dP1 = b13 * ((b11|E21) / (E21|E21));
	auto const dc12dP3 = -b11 * ((b13|E32) / (E32|E32));
	auto const dc12dP2 = -dc12dP1 - dc12dP3;

	// derivatives of the height:
	const auto dd00dP0 = -b00;
	const auto dd00dP1 = b00 * -((E21|E02) / (E21|E21));
	const auto dd00dP2 = b00 * ((E21|E01) / (E21|E21));
	const auto dd00dP3 = FVector::ZeroVector;

	const auto dd01dP0 = b01 * ((E02|-E21) / (E02|E02));
	const auto dd01dP1 = -b01;
	const auto dd01dP2 = b01 * ((E02|E01) / (E02|E02));
	const auto dd01dP3 = FVector::ZeroVector;

	const auto dd02dP0 = b02 * ((E01|E21) / (E01|E01));
	const auto dd02dP1 = b02 * ((E01|E02) / (E01|E01));
	const auto dd02dP2 = -b02;
	const auto dd02dP3 = FVector::ZeroVector;

	const auto dd11dP0 = FVector::ZeroVector;
	const auto dd11dP1 = -b11;
	const auto dd11dP2 = b11 * ((E32|E31) / (E32|E32));
	const auto dd11dP3 = b11 * ((E32|-E21) / (E32|E32));

	const auto dd12dP0 = FVector::ZeroVector;
	const auto dd12dP1 = b12 * ((E31|E32) / (E31|E31));
	const auto dd12dP2 = -b12;
	const auto dd12dP3 = b12 * ((E31|E21) / (E31|E31));

	const auto dd13dP0 = FVector::ZeroVector;
	const auto dd13dP1 = b13 * ((E21|-E32) / (E21|E21));
	const auto dd13dP2 = b13 * ((E21|E31) / (E21|E21));
	const auto dd13dP3 = -b13;

	// second derivatives of theta:
	const auto d2Theta_dP0dP0 = FRTMatrix3::CrossVec(n0, dd00dP0) / (d00 * d00) - dn0dP0 / d00;
	const auto d2Theta_dP0dP1 = FRTMatrix3::CrossVec(n0, dd00dP1) / (d00 * d00) - dn0dP1 / d00;
	const auto d2Theta_dP0dP2 = FRTMatrix3::CrossVec(n0, dd00dP2) / (d00 * d00) - dn0dP2 / d00;
	const auto d2Theta_dP0dP3 = FRTMatrix3::CrossVec(n0, dd00dP3) / (d00 * d00) - dn0dP3 / d00;

	const auto d2Theta_dP1dP0 = ((c02 / d01) * dn0dP0 + FRTMatrix3::CrossVec(n0, (d01 * dc02dP0 - c02 * dd01dP0)) / (d01 * d01)) + ((c12 / d11) * dn1dP0 + FRTMatrix3::CrossVec(n1, (d11 * dc12dP0 - c12 * dd11dP0)) / (d11 * d11));
	const auto d2Theta_dP1dP1 = ((c02 / d01) * dn0dP1 + FRTMatrix3::CrossVec(n0, (d01 * dc02dP1 - c02 * dd01dP1)) / (d01 * d01)) + ((c12 / d11) * dn1dP1 + FRTMatrix3::CrossVec(n1, (d11 * dc12dP1 - c12 * dd11dP1)) / (d11 * d11));
	const auto d2Theta_dP1dP2 = ((c02 / d01) * dn0dP2 + FRTMatrix3::CrossVec(n0, (d01 * dc02dP2 - c02 * dd01dP2)) / (d01 * d01)) + ((c12 / d11) * dn1dP2 + FRTMatrix3::CrossVec(n1, (d11 * dc12dP2 - c12 * dd11dP2)) / (d11 * d11));
	const auto d2Theta_dP1dP3 = ((c02 / d01) * dn0dP3 + FRTMatrix3::CrossVec(n0, (d01 * dc02dP3 - c02 * dd01dP3)) / (d01 * d01)) + ((c12 / d11) * dn1dP3 + FRTMatrix3::CrossVec(n1, (d11 * dc12dP3 - c12 * dd11dP3)) / (d11 * d11));

	const auto d2Theta_dP2dP0 = ((c01 / d02) * dn0dP0 + FRTMatrix3::CrossVec(n0, (d02 * dc01dP0 - c01 * dd02dP0)) / (d02 * d02) + ((c11 / d12)) * dn1dP0 + FRTMatrix3::CrossVec(n1, (d12 * dc11dP0 - c11 * dd12dP0)) / (d12 * d12));
	const auto d2Theta_dP2dP1 = ((c01 / d02) * dn0dP1 + FRTMatrix3::CrossVec(n0, (d02 * dc01dP1 - c01 * dd02dP1)) / (d02 * d02) + ((c11 / d12)) * dn1dP1 + FRTMatrix3::CrossVec(n1, (d12 * dc11dP1 - c11 * dd12dP1)) / (d12 * d12));
	const auto d2Theta_dP2dP2 = ((c01 / d02) * dn0dP2 + FRTMatrix3::CrossVec(n0, (d02 * dc01dP2 - c01 * dd02dP2)) / (d02 * d02) + ((c11 / d12)) * dn1dP2 + FRTMatrix3::CrossVec(n1, (d12 * dc11dP2 - c11 * dd12dP2)) / (d12 * d12));
	const auto d2Theta_dP2dP3 = ((c01 / d02) * dn0dP3 + FRTMatrix3::CrossVec(n0, (d02 * dc01dP3 - c01 * dd02dP3)) / (d02 * d02) + ((c11 / d12)) * dn1dP3 + FRTMatrix3::CrossVec(n1, (d12 * dc11dP3 - c11 * dd12dP3)) / (d12 * d12));	

	const auto d2Theta_dP3dP0 = FRTMatrix3::CrossVec(n1, dd13dP0) / (d13 * d13) - dn1dP0 / d13;
	const auto d2Theta_dP3dP1 = FRTMatrix3::CrossVec(n1, dd13dP1) / (d13 * d13) - dn1dP1 / d13;
	const auto d2Theta_dP3dP2 = FRTMatrix3::CrossVec(n1, dd13dP2) / (d13 * d13) - dn1dP2 / d13;
	const auto d2Theta_dP3dP3 = FRTMatrix3::CrossVec(n1, dd13dP3) / (d13 * d13) - dn1dP3 / d13;

	// Compute forces:

	// E = 1/2 C C
	// dE/dx = theta * dTheta_dX
	// f = -dE/dx
	// f = - theta * dTheta_dX
	const float KL = K * L;

	const auto d2Theta_dX_00 = FRTMatrix3::CrossVec(dTheta_dX0, dTheta_dX0);
	const auto d2Theta_dX_01 = FRTMatrix3::CrossVec(dTheta_dX0, dTheta_dX1);
	const auto d2Theta_dX_02 = FRTMatrix3::CrossVec(dTheta_dX0, dTheta_dX2);
	const auto d2Theta_dX_03 = FRTMatrix3::CrossVec(dTheta_dX0, dTheta_dX3);
	
	const auto d2Theta_dX_10 = FRTMatrix3::CrossVec(dTheta_dX1, dTheta_dX0);
	const auto d2Theta_dX_11 = FRTMatrix3::CrossVec(dTheta_dX1, dTheta_dX1);
	const auto d2Theta_dX_12 = FRTMatrix3::CrossVec(dTheta_dX1, dTheta_dX2);
	const auto d2Theta_dX_13 = FRTMatrix3::CrossVec(dTheta_dX1, dTheta_dX3);
	
	const auto d2Theta_dX_20 = FRTMatrix3::CrossVec(dTheta_dX2, dTheta_dX0);
	const auto d2Theta_dX_21 = FRTMatrix3::CrossVec(dTheta_dX2, dTheta_dX1);
	const auto d2Theta_dX_22 = FRTMatrix3::CrossVec(dTheta_dX2, dTheta_dX2);
	const auto d2Theta_dX_23 = FRTMatrix3::CrossVec(dTheta_dX2, dTheta_dX3);
	
	const auto d2Theta_dX_30 = FRTMatrix3::CrossVec(dTheta_dX3, dTheta_dX0);
	const auto d2Theta_dX_31 = FRTMatrix3::CrossVec(dTheta_dX3, dTheta_dX1);
	const auto d2Theta_dX_32 = FRTMatrix3::CrossVec(dTheta_dX3, dTheta_dX2);
	const auto d2Theta_dX_33 = FRTMatrix3::CrossVec(dTheta_dX3, dTheta_dX3);
	
	// dfdx
	const auto df0dP0 = -KL * (Theta * d2Theta_dP0dP0 + d2Theta_dX_00);
	const auto df0dP1 = -KL * (Theta * d2Theta_dP0dP1 + d2Theta_dX_01);
	const auto df0dP2 = -KL * (Theta * d2Theta_dP0dP2 + d2Theta_dX_02);
	const auto df0dP3 = -KL * (Theta * d2Theta_dP0dP3 + d2Theta_dX_03);

	const auto df1dP0 = -KL * (Theta * d2Theta_dP1dP0 + d2Theta_dX_10);
	const auto df1dP1 = -KL * (Theta * d2Theta_dP1dP1 + d2Theta_dX_11);
	const auto df1dP2 = -KL * (Theta * d2Theta_dP1dP2 + d2Theta_dX_12);
	const auto df1dP3 = -KL * (Theta * d2Theta_dP1dP3 + d2Theta_dX_13);

	const auto df2dP0 = -KL * (Theta * d2Theta_dP2dP0 + d2Theta_dX_20);
	const auto df2dP1 = -KL * (Theta * d2Theta_dP2dP1 + d2Theta_dX_21);
	const auto df2dP2 = -KL * (Theta * d2Theta_dP2dP2 + d2Theta_dX_22);
	const auto df2dP3 = -KL * (Theta * d2Theta_dP2dP3 + d2Theta_dX_23);

	const auto df3dP0 = -KL * (Theta * d2Theta_dP3dP0 + d2Theta_dX_30);
	const auto df3dP1 = -KL * (Theta * d2Theta_dP3dP1 + d2Theta_dX_31);
	const auto df3dP2 = -KL * (Theta * d2Theta_dP3dP2 + d2Theta_dX_32);
	const auto df3dP3 = -KL * (Theta * d2Theta_dP3dP3 + d2Theta_dX_33);

	// fill in
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dfdx[3 * V_idx[0] + i][3 * V_idx[0] + j] += df0dP0[i][j];
			dfdx[3 * V_idx[0] + i][3 * V_idx[1] + j] += df0dP1[i][j];
			dfdx[3 * V_idx[0] + i][3 * V_idx[2] + j] += df0dP2[i][j];
			dfdx[3 * V_idx[0] + i][3 * V_idx[3] + j] += df0dP3[i][j];

			dfdx[3 * V_idx[1] + i][3 * V_idx[0] + j] += df1dP0[i][j];
			dfdx[3 * V_idx[1] + i][3 * V_idx[1] + j] += df1dP1[i][j];
			dfdx[3 * V_idx[1] + i][3 * V_idx[2] + j] += df1dP2[i][j];
			dfdx[3 * V_idx[1] + i][3 * V_idx[3] + j] += df1dP3[i][j];

			dfdx[3 * V_idx[2] + i][3 * V_idx[0] + j] += df2dP0[i][j];
			dfdx[3 * V_idx[2] + i][3 * V_idx[1] + j] += df2dP1[i][j];
			dfdx[3 * V_idx[2] + i][3 * V_idx[2] + j] += df2dP2[i][j];
			dfdx[3 * V_idx[2] + i][3 * V_idx[3] + j] += df2dP3[i][j];

			dfdx[3 * V_idx[3] + i][3 * V_idx[0] + j] += df3dP0[i][j];
			dfdx[3 * V_idx[3] + i][3 * V_idx[1] + j] += df3dP1[i][j];
			dfdx[3 * V_idx[3] + i][3 * V_idx[2] + j] += df3dP2[i][j];
			dfdx[3 * V_idx[3] + i][3 * V_idx[3] + j] += df3dP3[i][j];
		}
	}

	// compute damping forces and v derivatives:
	// fd = -d * dTheta/dt * dTheta/dx:
	const float DL = D * L;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dddv[3 * V_idx[0] + i][3 * V_idx[0] + j] += -DL * d2Theta_dX_00[i][j];
			dddv[3 * V_idx[0] + i][3 * V_idx[1] + j] += -DL * d2Theta_dX_01[i][j];
			dddv[3 * V_idx[0] + i][3 * V_idx[2] + j] += -DL * d2Theta_dX_02[i][j];
			dddv[3 * V_idx[0] + i][3 * V_idx[3] + j] += -DL * d2Theta_dX_03[i][j];
			
			dddv[3 * V_idx[1] + i][3 * V_idx[0] + j] += -DL * d2Theta_dX_10[i][j];
			dddv[3 * V_idx[1] + i][3 * V_idx[1] + j] += -DL * d2Theta_dX_11[i][j];
			dddv[3 * V_idx[1] + i][3 * V_idx[2] + j] += -DL * d2Theta_dX_12[i][j];
			dddv[3 * V_idx[1] + i][3 * V_idx[3] + j] += -DL * d2Theta_dX_13[i][j];
			
			dddv[3 * V_idx[2] + i][3 * V_idx[0] + j] += -DL * d2Theta_dX_20[i][j];
			dddv[3 * V_idx[2] + i][3 * V_idx[1] + j] += -DL * d2Theta_dX_21[i][j];
			dddv[3 * V_idx[2] + i][3 * V_idx[2] + j] += -DL * d2Theta_dX_22[i][j];
			dddv[3 * V_idx[2] + i][3 * V_idx[3] + j] += -DL * d2Theta_dX_23[i][j];
			
			dddv[3 * V_idx[3] + i][3 * V_idx[0] + j] += -DL * d2Theta_dX_30[i][j];
			dddv[3 * V_idx[3] + i][3 * V_idx[1] + j] += -DL * d2Theta_dX_31[i][j];
			dddv[3 * V_idx[3] + i][3 * V_idx[2] + j] += -DL * d2Theta_dX_32[i][j];
			dddv[3 * V_idx[3] + i][3 * V_idx[3] + j] += -DL * d2Theta_dX_33[i][j];
		}
	}
	
	// dddx = -kd * d2c/dpi_dpj dc/dt
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dddx[3 * V_idx[0] + i][3 * V_idx[0] + j] += - DL * dTheta_dt * d2Theta_dP0dP0[i][j];
			dddx[3 * V_idx[0] + i][3 * V_idx[1] + j] += - DL * dTheta_dt * d2Theta_dP0dP1[i][j];
			dddx[3 * V_idx[0] + i][3 * V_idx[2] + j] += - DL * dTheta_dt * d2Theta_dP0dP2[i][j];
			dddx[3 * V_idx[0] + i][3 * V_idx[3] + j] += - DL * dTheta_dt * d2Theta_dP0dP3[i][j];
			
			dddx[3 * V_idx[1] + i][3 * V_idx[0] + j] += - DL * dTheta_dt * d2Theta_dP1dP0[i][j];
			dddx[3 * V_idx[1] + i][3 * V_idx[1] + j] += - DL * dTheta_dt * d2Theta_dP1dP1[i][j];
			dddx[3 * V_idx[1] + i][3 * V_idx[2] + j] += - DL * dTheta_dt * d2Theta_dP1dP2[i][j];
			dddx[3 * V_idx[1] + i][3 * V_idx[3] + j] += - DL * dTheta_dt * d2Theta_dP1dP3[i][j];
			
			dddx[3 * V_idx[2] + i][3 * V_idx[0] + j] += - DL * dTheta_dt * d2Theta_dP2dP0[i][j];
			dddx[3 * V_idx[2] + i][3 * V_idx[1] + j] += - DL * dTheta_dt * d2Theta_dP2dP1[i][j];
			dddx[3 * V_idx[2] + i][3 * V_idx[2] + j] += - DL * dTheta_dt * d2Theta_dP2dP2[i][j];
			dddx[3 * V_idx[2] + i][3 * V_idx[3] + j] += - DL * dTheta_dt * d2Theta_dP2dP3[i][j];
			
			dddx[3 * V_idx[3] + i][3 * V_idx[0] + j] += - DL * dTheta_dt * d2Theta_dP3dP0[i][j];
			dddx[3 * V_idx[3] + i][3 * V_idx[1] + j] += - DL * dTheta_dt * d2Theta_dP3dP1[i][j];
			dddx[3 * V_idx[3] + i][3 * V_idx[2] + j] += - DL * dTheta_dt * d2Theta_dP3dP2[i][j];
			dddx[3 * V_idx[3] + i][3 * V_idx[3] + j] += - DL * dTheta_dt * d2Theta_dP3dP3[i][j];
		}
	}
}