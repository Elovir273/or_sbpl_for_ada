#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <cmath>

using namespace or_sbpl_for_ada;

void multiply(double a[3][3], double b[3][3], double c[3][3] ) {
  int k=0,i=0,j=0;
  for(i=0;i<3;++i) {
    for(j=0;j<3;++j) {
      c[i][j]=0;
      for(k=0;k<3;++k) {
        c[i][j]=c[i][j]+(a[i][k]*b[k][j]);
      }
    }
  }
}

OpenRAVE::Transform WorldCoordinate::toTransform() const {
  
  // Rotation
  
  double Mphi[3][3];
  double Mtheta[3][3];
  double Mpsi[3][3];

  Mphi[0][0]=1.;
  Mphi[0][1]=0.;
  Mphi[0][2]=0.;
  Mphi[1][0]=0.;
  Mphi[1][1]=OpenRAVE::RaveCos(phi);
  Mphi[1][2]=-OpenRAVE::RaveSin(phi);
  Mphi[3][0]=0.;
  Mphi[3][1]=OpenRAVE::RaveSin(phi);
  Mphi[3][2]=OpenRAVE::RaveCos(phi);

  Mtheta[0][0]=OpenRAVE::RaveCos(theta);
  Mtheta[0][1]=0.;
  Mtheta[0][2]=OpenRAVE::RaveSin(theta);
  Mtheta[1][0]=0.;
  Mtheta[1][1]=1.;
  Mtheta[1][2]=0.;
  Mtheta[3][0]=-OpenRAVE::RaveSin(theta);
  Mtheta[3][1]=0.;
  Mtheta[3][2]=OpenRAVE::RaveCos(theta);

  Mpsi[0][0]=OpenRAVE::RaveCos(psi);
  Mpsi[0][1]=-OpenRAVE::RaveSin(psi);
  Mpsi[0][2]=0.;
  Mpsi[1][0]=OpenRAVE::RaveSin(psi);
  Mpsi[1][1]=OpenRAVE::RaveCos(psi);
  Mpsi[1][2]=0.;
  Mpsi[3][0]=0.;
  Mpsi[3][1]=0.;
  Mpsi[3][2]=1.;

  double res_temp[3][3];
  double res[3][3];
  multiply(Mpsi,Mtheta,res);
  multiply(res_temp,Mphi,res);

  OpenRAVE::RaveTransformMatrix<double> R;
  R.rotfrommat(res[0][0], res[0][1], res[0][2],
   res[1][0], res[1][1] ,res[1][2] ,
   res[2][0],res[2][1] ,res[2][2] );

    // Translation
  OpenRAVE::RaveVector<double> t(x, y, z);

    // Now put them together
  OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(R), t);

  return transform;
}

