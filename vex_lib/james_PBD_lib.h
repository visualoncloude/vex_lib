

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
                        matrix3 rot0T = matrix3_transpose(qconvert(q0));
                        matrix3 rot1T = matrix3_transpose(qconvert(q1));
                        ballJointInfo[0]= rot0T * (ballJointPosition - x0);
                        ballJointInfo[1] = rot1T * (ballJointPosition - x1);
                        ballJointInfo[2] = ballJointPosition;
                        ballJointInfo[3] = ballJointPosition;
                    }


void update_BallJoint(vector  x0 , x1 ;
                    vector4 q0, q1 ;
                    vector ballJointPosition;
                    vector ballJointInfo[])
                    {
                        matrix3 rot0T = matrix3_transpose(qconvert(q0));
                        matrix3 rot1T = matrix3_transpose(qconvert(q1));
                        ballJointInfo[2]= rot0T * (ballJointPosition[0] - x0)  + x0;
                        ballJointInfo[3] = rot1T * (ballJointPosition[1] - x1) + x1;
                       
                    }



void DistanceConstraint( vector  p0, p1 ; 
                    float invMass0, invMass1;
                    float  restLength;
                    float compressionStiffness, stretchStiffness;
                    vector corr0, corr1)
        {                               
                float wSum = invMass0 + invMass1;
                vector n = p1 - p0;
                float d = length(n);
                 n  = normalize(n);
                
                vector corr;
                if (d < restLength)
                        corr = compressionStiffness * n * (d - restLength) / wSum;
                else
                        corr = stretchStiffness * n * (d - restLength) / wSum;
        
                corr0 =  invMass0 * corr;
                corr1 = -invMass1 * corr;
                
        }
        
/*
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
*/

