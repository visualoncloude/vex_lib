
void computeMatrixK( vector connector;
	         float invMass;
	         vector x;
	         matrix3 inertiaInverseW;
	         matrix3 K)
{
	if (invMass != 0.0)
	{
		vector v = connector - x;
		float  a = v[0];
		float b = v[1];
		float c = v[2];

		// J is symmetric
		float j11 = inertiaInverseW.xx;
		float j12 = inertiaInverseW.xy;
		float j13 = inertiaInverseW.xz;
		float j22 = inertiaInverseW.yy;
		float j23 = inertiaInverseW.yz;
		float j33 = inertiaInverseW.zz;

		K.xx = c*c*j22 - b*c*(j23 + j23) + b*b*j33 + invMass;
		K.xy = -(c*c*j12) + a*c*j23 + b*c*j13 - a*b*j33;
		K.xz = b*c*j12 - a*c*j22 - b*b*j13 + a*b*j23;
		K.yx = K.xy;
		K.yy = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + invMass;
		K.yz = -(b*c*j11) + a*c*j12 + a*b*j13 - a*a*j23;
		K.zx = K.xz;
		K.zy = K.yz;
		K.zz = b*b*j11 - a*b*(j12 + j12) + a*a*j22 + invMass;
	}
	else
	{
		K = set(0,0,0,0,0,0,0,0,0);
	}
}




vector center_edge(vector p0 , p1)
{
    vector  center_edge = (p0 + p1)/2;
    return center_edge;
}

void init_BallJoint(vector  x0 , x1 ;
                    vector4 q0, q1 ;
                    vector ballJointPosition;
                    vector ballJointInfo[])
                    {
			matrix3 q0_m =  qconvert(q0);
			matrix3 q1_m =  qconvert(q1);                       
			matrix3 rot0T = matrix3_transpose(q0_m);
                        matrix3 rot1T = matrix3_transpose(q1_m);
                        ballJointInfo[0] = rot0T * (ballJointPosition - x0);
                        ballJointInfo[1] = rot1T * (ballJointPosition - x1);
                        ballJointInfo[2] = ballJointPosition-  ballJointInfo[0];
                        ballJointInfo[3] = ballJointPosition- ballJointInfo[1];
                    }


void update_BallJoint(vector  x0 , x1 ;
                    vector4 q0, q1 ;
                    vector ballJointInfo[])
                    {
			//matrix3 q0_m =  qconvert(q0);
			//matrix3 q1_m =  qconvert(q1);                         
			matrix3 rot0 = qconvert(q0);
                        matrix3 rot1 =  qconvert(q1); 
			ballJointInfo[2]= rot0 * ballJointInfo[0]  + x0;
			ballJointInfo[3] = rot1 * ballJointInfo[1]  + x1;

                    }

void solve_BallJoint(vector  x0 , x1 ;
		     float  invMass0 , invMass1;
		     vector4 q0, q1 ;
	      	     matrix3 inertiaInverseW0 , inertiaInverseW1;
                     vector ballJointPosition;
                     vector ballJointInfo[];
                     vector corr_x0,corr_x1;
                     vector4 corr_q0,corr_q0 )
                    {
                        vector connector0 = ballJointInfo[2];
			vector connector1 = ballJointInfo[3];
			matrix3 k1,k2;
			computeMatrixK(connector0 , invMass0, x0,inertiaInverseW0,k1 );
			computeMatrixK(connector1 , invMass1, x1,inertiaInverseW0,k2 );
			
			vector pt = (K1 + K2).llt().solve(connector1 - connector0);
			
			if (invMass0 != 0.0)
			{			
				vector r0 = connector0 - x0;				
				corr_x0 = invMass0*pt;
				vector ot = (inertiaInverseW0 * cross(r0,pt));				
				vector4 otQ = set(0.0, ot.x, ot.y, ot.z);
				corr_q1 = otQ*q0*0.5;
			}
			if (invMass0 != 0.0)
			{			
				vector r1 = connector0 - x1;				
				corr_x1 = -invMass0*pt;
				vector ot = (inertiaInverseW0 * cross(r1,pt));				
				vector4 otQ = set(0.0, ot.x, ot.y, ot.z);
				corr_q1 = otQ*q1*0.5;
			}
                     
		     
		     }














