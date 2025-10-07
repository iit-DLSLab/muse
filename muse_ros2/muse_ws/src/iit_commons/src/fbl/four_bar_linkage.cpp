#include <cmath>
#include <iit/commons/four_bar_linkage.h>


iit::commons::FBLStatus::FBLStatus(const FBLConsts& p) : theta(0.0), extension(0.0), jac(0.0), consts(p)
{
    // The triangle CJK, which is constant
    ck2 = p.cj*p.cj + p.jk*p.jk;
    ck  = sqrt(ck2);
    ckh = acos(p.jk/ck);

    // The triangle CAK, which is constant
    cka = ckh + p.akh;
    ak2 = p.ak * p.ak;
    ca2 = ak2 + ck2 - 2*p.ak*ck*cos(cka);
    ca  = sqrt(ca2);
    cak = acos( (ca2 + ak2 - ck2) / (2*ca*p.ak) );

    // Other squares
    bk2 = p.bk * p.bk;
    ar2 = p.ar * p.ar;
    br2 = p.br * p.br;

    angle_KB_lleg_xaxis = p.angle_KB_lleg_xaxis;
    cyl_retracted = p.cyl_retracted;
}



void iit::commons::FBLStatus::updateGeometry()
{
    const double bkh = theta;

    // The triangle ABK

    const double akb = consts.akh + bkh;
    const double ab2 = ak2 + bk2 - 2*consts.ak*consts.bk*cos(akb);
    const double ab  = sqrt(ab2);
    const double kba = acos( (bk2 + ab2 - ak2) / (2*consts.bk*sqrt(ab2)) );
    //double kab = M_PI - kba - akb;

    const double ab_J_th  = consts.ak*consts.bk*sin(akb) / ab;
    const double kba_J_ab = (bk2 - ab2 - ak2) / (2*consts.bk*ab2) / sin(kba);
    const double kba_J_th = kba_J_ab * ab_J_th;


    // The triangle ABR

    const double arb = acos( (ar2 + br2 - ab2) / (2*consts.ar*consts.br ) );
    const double abr = (M_PI - arb) / 2;

    const double arb_J_ab = ab / (consts.ar*consts.br) / sin(arb);
    const double abr_J_ab = - arb_J_ab/2;
    const double abr_J_th = abr_J_ab * ab_J_th;


    // The triangle BKR
    double kbr = 0;
    double kbr_J_th = 0;
    if(akb > M_PI) {
        kbr = abr - kba;
        kbr_J_th = abr_J_th - kba_J_th;
    } else {
        kbr = abr + kba;
        kbr_J_th = abr_J_th + kba_J_th;
    }

    const double kr2 = bk2 + br2 - 2*consts.bk*consts.br*cos(kbr);
    const double kr = sqrt(kr2);
    const double bkr = acos( (bk2 + kr2 - br2) / (2*consts.bk*kr) );
    //const double krb = M_PI - kbr - bkr;

    const double kr_J_kbr = consts.bk*consts.br*sin(kbr) / kr;
    const double kr_J_th  = kr_J_kbr * kbr_J_th;

    const double bkr_J_kr = (bk2 - kr2 - br2) / (2*consts.bk*kr2) / sin(bkr);
    const double bkr_J_th = bkr_J_kr * kr_J_th;


    // The triangle CKR

    const double ckr = bkh - bkr - ckh;
    const double cr2 = ck*ck + kr2 - 2*ck*kr*cos(ckr); // cylinder extension (squared)
    extension = sqrt(cr2) - cyl_retracted;
    //const double crk = acos( (cr2 + kr2 - ck2) / (2*cylinder*kr) );

    // The time derivative of 'bkh' is the same as 'theta'
    // 'ckh' is a constant. Thus the Jacobian is:
    const double ckr_J_th = 1 - bkr_J_th;

    const double cr_J_ckr = ck*kr*sin(ckr) / extension;
    const double cr_J_kr  = (kr - ck*cos(ckr)) / extension;

    jac = cr_J_ckr * ckr_J_th + cr_J_kr * kr_J_th; // 'cr_J_th'
}

void iit::commons::FBLStatus::setExtension(double cyl)
{
    extension = cyl;

    // Triangle CAR
    double car = acos( (ca2 + ar2 - cyl*cyl) / (2*ca*consts.ar) );

    // Triangle RAK
    double rak = cak - car;
    double kr2 = ak2 + ar2 - 2*consts.ak*consts.ar*cos(rak);
    double kr  = sqrt( kr2 );
    double akr = acos( (ak2 + kr2 - ar2) / (2*consts.ak*kr) );

    // Triangle RBK
    double rkb = acos( (kr2 + bk2 - br2) / (2*kr*consts.bk) );

    theta = akr + rkb - consts.akh;
}


void iit::commons::FBLStatus::computeNumericJacobian(double q)
{
    double deltaQ = 0.0001;
    setAngle(kneeAngleToFBLAngle(q+deltaQ));
    double tmp = extension;
    setAngle(kneeAngleToFBLAngle(q));
    double tmp2 = extension;
    jac_num = (tmp - tmp2) / deltaQ;
}





