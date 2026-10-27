#include "test_case.h"
#include "../survive_config.h"
#include <math.h>
#include <string.h>

// Independent Hamilton/rotation-matrix physical uncertainty oracle.
static void hamilton(FLT o[4],const FLT a[4],const FLT b[4]) {
    o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
static void matrix(FLT R[9],const FLT q[4]) {
    FLT w=q[0],x=q[1],y=q[2],z=q[3];
    FLT values[9]={1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};
    memcpy(R,values,sizeof(values));
}
static void physical_pose(SurvivePose out[2],const SurvivePose nominal[2],const FLT error[12]) {
    for(int camera=0;camera<2;camera++) {
        out[camera]=nominal[camera];
        for(int i=0;i<3;i++)out[camera].Pos[i]+=error[6*camera+i];
        const FLT *v=error+6*camera+3;FLT a=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]),dq[4]={cos(a/2)},q[4];
        for(int i=0;i<3;i++)dq[i+1]=a>0?v[i]*sin(a/2)/a:0;
        hamilton(q,nominal[camera].Rot,dq);memcpy(out[camera].Rot,q,sizeof(q));
    }
}
static void inverse_reference(FLT out[14],const SurvivePose input[2],const SurvivePose *frame) {
    FLT F[9];matrix(F,frame->Rot);
    for(int camera=0;camera<2;camera++) {
        FLT R[9],p[3]={0},qi[4],q[4];matrix(R,input[camera].Rot);
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)p[i]-=R[3*j+i]*input[camera].Pos[j];
        for(int i=0;i<3;i++) {out[7*camera+i]=frame->Pos[i];for(int j=0;j<3;j++)out[7*camera+i]+=F[3*i+j]*p[j];}
        qi[0]=input[camera].Rot[0];for(int i=1;i<4;i++)qi[i]=-input[camera].Rot[i];
        hamilton(q,frame->Rot,qi);memcpy(out+7*camera+3,q,sizeof(q));
    }
}


TEST(PoseCovariance, InverseJacobianMatchesPoseOperation) {
    for (int mode=0;mode<4;mode++) {
        SurvivePose pose={.Pos={2,-3,4},.Rot={.7,-.2,.4,.5}};
        FLT norm=quatmagnitude(pose.Rot),scale=mode&2?1.3:1;
        for(int i=0;i<4;i++)pose.Rot[i]*=(mode&1?-scale:scale)/norm;
        CN_CREATE_STACK_MAT(G,7,7);
        ASSERT_EQ(survive_pose_inverse_jacobian(&G,&pose),true);
        for(int col=0;col<7;col++) {
            SurvivePose plus=pose,minus=pose,a,b;
            FLT epsilon=sizeof(FLT)==sizeof(float)?1e-3:1e-6;
            ((FLT*)&plus)[col]+=epsilon;((FLT*)&minus)[col]-=epsilon;
            InvertPose(&a,&plus);InvertPose(&b,&minus);
            for(int row=0;row<7;row++)ASSERT_GT(sizeof(FLT)==sizeof(float)?2e-3:1e-7,fabs((((FLT*)&a)[row]-((FLT*)&b)[row])/(2*epsilon)-cnMatrixGet(&G,row,col)));
        }
    }
    SurvivePose invalid={0};CN_CREATE_STACK_MAT(G,7,7);
    ASSERT_EQ(survive_pose_inverse_jacobian(&G,&invalid),false);
    ASSERT_EQ(cn_is_finite(&G),true);
    return 0;
}

TEST(PoseCovariance, JointInverseKeepsPhysicalLeverArmsAndCrossCovariance) {
    for(int mode=0;mode<8;mode++) {
        SurvivePose nominal[2]={ {.Pos={2.,-.3,.7},.Rot={1}}, {.Pos={-1.5,.7,2.2},.Rot={.8,.2,-.3,.4}} };
        FLT norm=sqrt(.8*.8+.2*.2+.3*.3+.4*.4);for(int i=0;i<4;i++)nominal[1].Rot[i]/=norm;
        if(mode&1)for(int camera=0;camera<2;camera++)for(int i=0;i<4;i++)nominal[camera].Rot[i]*=-1;
        SurvivePose frame={.Rot={1}};
        if(mode&2)frame=(SurvivePose){.Pos={.2,-.3,.1},.Rot={cos(.43/2),0,0,sin(.43/2)}};
        FLT source_jac[14*12],inverse_jac[14*12],L[144]={0},C[144]={0};
        const FLT epsilon=sizeof(FLT)==sizeof(float)?1e-3:1e-6;
        for(int col=0;col<12;col++) {
            FLT e[12]={0},plus[14],minus[14];SurvivePose p[2],m[2];e[col]=epsilon;physical_pose(p,nominal,e);e[col]=-epsilon;physical_pose(m,nominal,e);
            inverse_reference(plus,p,&frame);inverse_reference(minus,m,&frame);
            for(int row=0;row<14;row++) {
                source_jac[12*row+col]=(((FLT*)p)[row]-((FLT*)m)[row])/(2*epsilon);
                inverse_jac[12*row+col]=(plus[row]-minus[row])/(2*epsilon);
            }
        }
        for(int i=0;i<12;i++){L[12*i+i]=i%6<3?.001:.01;for(int j=0;j<i;j++)L[12*i+j]=.0002*sin(3*i+j);}
        for(int i=0;i<12;i++)for(int j=0;j<12;j++)for(int k=0;k<12;k++)C[12*i+j]+=L[12*i+k]*L[12*j+k];
        FLT source_values[196]={0},expected[196]={0},output_values[196]={0};
        for(int i=0;i<14;i++)for(int j=0;j<14;j++)for(int a=0;a<12;a++)for(int b=0;b<12;b++) {
            source_values[14*i+j]+=source_jac[12*i+a]*C[12*a+b]*source_jac[12*j+b];
            expected[14*i+j]+=inverse_jac[12*i+a]*C[12*a+b]*inverse_jac[12*j+b];
        }
        CnMat source=cnMat(14,14,source_values),out=cnMat(14,14,mode&4?source_values:output_values);
        survive_covariance_invert_poses(&out,nominal,&source,2);
        if(mode&2)survive_covariance_apply_pose_to_poses(&out,&frame,&out,2);
        for(int row=0;row<14;row++)for(int col=0;col<14;col++)ASSERT_GT(sizeof(FLT)==sizeof(float)?1e-6:1e-10,fabs(cnMatrixGet(&out,row,col)-expected[14*row+col]));
    }
    return 0;
}

TEST(PoseCovariance, SceneNormalizationTransportsInterLighthouseCovariance) {
    SurviveContext ctx={.activeLighthouses=2,.log_level=-1};
    config_group temporary={.write_lock=OGCreateMutex(),.ctx=&ctx};ctx.temporary_config_values=&temporary;
    ctx.bsd[0].BaseStationID=1;ctx.bsd[1].BaseStationID=2;
    SurvivePose original[2]={ {.Pos={2,-.3,.7},.Rot={1}}, {.Pos={-1.5,.7,2.2},.Rot={1}} };
    SurvivePose poses[2];memcpy(poses,original,sizeof(poses));SurvivePose object={.Rot={1}};
    FLT data[196]={0},before[196],G[196]={0},expected[196]={0};
    for(int i=0;i<14;i++)data[14*i+i]=.001;
    data[7]=data[98]=.0002;data[1*14+9]=data[9*14+1]=-.0001;
    data[3*14+12]=data[12*14+3]=.0001;
    memcpy(before,data,sizeof(data));CnMat R=cnMat(14,14,data);
    PoserData_normalize_scene(&ctx,poses,2,&object,&R);
    SurvivePose inverse=InvertPoseRtn(&original[0]),frame;ApplyPoseToPose(&frame,&poses[0],&inverse);
    FLT F[9];matrix(F,frame.Rot);
    for(int camera=0;camera<2;camera++) {
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)G[14*(7*camera+i)+7*camera+j]=F[3*i+j];
        for(int col=0;col<4;col++){FLT basis[4]={0},q[4];basis[col]=1;hamilton(q,frame.Rot,basis);for(int row=0;row<4;row++)G[14*(7*camera+3+row)+7*camera+3+col]=q[row];}
    }
    for(int i=0;i<14;i++)for(int j=0;j<14;j++)for(int a=0;a<14;a++)for(int b=0;b<14;b++)expected[14*i+j]+=G[14*i+a]*before[14*a+b]*G[14*j+b];
    for(int i=0;i<196;i++)ASSERT_GT(sizeof(FLT)==sizeof(float)?1e-8:1e-12,fabs(data[i]-expected[i]));
    for(int i=0;i<temporary.used_entries;i++){free(temporary.config_entries[i].tag);free(temporary.config_entries[i].data);}free(temporary.config_entries);OGDeleteMutex(temporary.write_lock);
    return 0;
}
