#include "Robot.h"


Robot::Robot()
{
    // A MODIFIER
    uint nb_joint = 5;      // définition du nombre d'articulation
    uint nb_body = 5;       // définition du nombre de corps
    TJoint.resize(nb_joint);   // on définit que le vecteur TJoint (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    TLink.resize(nb_body);     // on définit que le vecteur TLink  (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)    
    TStatic.resize(nb_joint);  // on définit que le vecteur TStatic (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    
    qmin.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
    qmax.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
    
    TStatic[0] = Transformation (0,0,0,0,0,0);                // on définit la premiere transformation, les 6 paramètres sont : Tx, Ty, Tz, Roll, Pitch, Yaw  (angle en degré)
    qmin[0] = 0;
    qmax[0] = 180;
    
    qmin[1] = 15;
    qmax[1] = 165;
    
    qmin[2] = 0;
    qmax[2] = 180;
    
    qmin[3] = 0;
    qmax[3] = 180;
    
    qmin[4] = 0;
    qmax[4] = 180;
    
}

// renvoi la pose de l'effecteur en fonction des variables articulaires Q.
Transformation Robot::ModGeoDirect( const VECTOR& Q)
{
    double q1 = Q.data[0];
    double q2 = Q.data[1];
    double q3 = Q.data[2];
    double q4 = Q.data[3];
    double q5 = Q.data[4];

    Transformation T01, T12, T23, T34,T45,T56,Teff;
    
    
    T01 = Transformation(0,0,0.01,0,0,0)*RotZ(q1);
    T12 = Transformation(0,-0.002,0.072,-90,0,0)*RotX(q2);
    T23 = Transformation(0,0,0.125,-90,0,0)*RotX(q3);
    T34 = Transformation(0,0,0.125,-90,0,0)*RotX(q4);
    T45 = Transformation(0,0,0.06,0,0,90)*RotZ(-q5);
    T56 = Transformation(0,0,0.1,0,0,0);

    // A COMPLETER
     Teff = T01*T12*T23*T34*T45*T56;
     
     return Teff;
}

// Cette fonction calcule les valeurs articulaires pour que l'effecteur atteigne le point définit par le vecteur pos.
// Si le point n'est pas accessible, la fonction renvoie false
bool  Robot::ModGeoInverse( const Eigen::Matrix<double,3,1> & pos,
                            VECTOR & Q)
{    
    Q.data.resize(6);    
    double X = pos(0);
    double Y = pos(1);
    double Z = pos(2);
    
    double q1 = atan2(Y,X);
    

    // A COMPLETER
    

    
    // ne pas oublier de vérifier que les valeurs sont dans les limites articulaires
    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Q.data[i] < qmin[i]) Q.data[i] = qmin[i];
        if (Q.data[i] > qmax[i]) Q.data[i] = qmax[i];
    }    
    
    // Renvoi vrai si faisable
    return true;
}

// Renvoie la jacobienne (position en 3D d'un point de l'effecteur) pour un vecteur articulaire
Eigen::Matrix<double,3,5> Robot::ComputeJacobian( const VECTOR& Q)
{
    Eigen::Matrix<double,3,5> out;
    double l1 = 0.072;
    double l2 = 0.125;
    double l3 = 0.125;
    double l4 = 0.06;
    double l5 = 0.1; // Distance entre dérnière rotation et fin de la pince
    
    double q1 = Q.data[0];
    double q2 = Q.data[1];
    double q3 = Q.data[2];
    double q4 = Q.data[3];
    double q5 = Q.data[4];
    
    out(0,0) = -sin(q1)*(l2*cos(q2)+l3*cos(q2+q3)+(l4+l5)*cos(q2+q3+q4));
    out(0,1) = -cos(q1)*(l2*sin(q2)+l3*sin(q2+q3)+(l4+l5)*sin(q2+q3+q4));
    out(0,2) = -cos(q1)*(l3*sin(q2+q3)+(l4+l5)*sin(q2+q3+q4));
    out(0,3) = -cos(q1)*((l4+l5)*sin(q2+q3+q4));
    out(0,4) = 0;
    
    out(1,0) = cos(q1)*(l2*cos(q2)+l3*cos(q2+q3)+(l4+l5)*cos(q2+q3+q4));
    out(1,1) =-sin(q1)*(l2*sin(q2)+l3*sin(q2+q3)+(l4+l5)*sin(q2+q3+q4));
    out(1,2) =-sin(q1)*(l3*sin(q2+q3)+(l4+l5)*sin(q2+q3+q4));
    out(1,3) =-sin(q1)*(l4*sin(q2+q3+q4));
    out(1,4) = 0;
    
    out(2,0) = 0;
    out(2,1) = l2*cos(q2)+l3*cos(q2+q3)+(l4+l5)*cos(q2+q3+q4);
    out(2,2) = l3*cos(q2+q3)+(l4+l5)*cos(q2+q3+q4);
    out(2,3) = (l4+l5)*cos(q2+q3+q4);
    out(2,4) = 0;

    return out;
}



// Calcule le vecteur articulaire suivant permettant de se rapprocher de la cible
// entrees
//      Qin : Vecteur articulaire initial
//      CurrentPosition : Position (vecteur 3D) actuelle de l'effecteur (par modèle ou par mesure)
//      DesiredPosition : Position (vecteur 3D) désirée de l'effecteur
// sorties
//      Qout : Vecteur articulaire permettant de se rapprocher de la cible
//      retourne la distance entre la position actuelle et désirée
double Robot::ComputeControl(   const VECTOR& Qin,
                                const Eigen::Matrix<double,3,1>& CurrentPosition,
                                const Eigen::Matrix<double,3,1>& DesiredPosition,
                                VECTOR& Qout)
{
    // on récupère la jacobienne 
    Eigen::Matrix<double,3,5> Jacobian = ComputeJacobian(Qin);
    
    Eigen::Matrix<double,3,1> vitesse;
    Eigen::Matrix<double,5,1> dq;
    double deltat = 0.001;

    vitesse = DesiredPosition-CurrentPosition;
    dq = (Jacobian.transpose()*(Jacobian*Jacobian.transpose()).inverse())*vitesse;
    
    std::cout<<"Jacobian = "<< Jacobian <<std::endl;
    std::cout<<"VitesseEffecteur = "<< vitesse <<std::endl;
    
    
    
    
  
    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
        Qout.data[i] = Qin.data[i] + dq(i) * deltat;  
    
    

    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Qout.data[i] < qmin[i]) Qout.data[i] = qmin[i];
        if (Qout.data[i] > qmax[i]) Qout.data[i] = qmax[i];
        std::cout<<"Qout.data["<<i<<" ] = "<<Qout.data[i] <<std::endl;
    }           
    
    return (DesiredPosition-CurrentPosition).norm();  // A COMPLETER
}
