#include <BasicLinearAlgebra.h>

using namespace BLA;

double R_beam = 0.3;
double R_wheel = 0.1;
double R_acc = 0.15;
double M_beam = 0.75;
double M_wheel = 0.01;
double g = 9.81;
double I_beam = (1 / 12) * M_beam * pow(2 * R_beam, 2);
double I_wheel = (1 / 2) * M_wheel * pow(R_wheel, 2);
double St_acc = 0.0;
double Mot_delay = 5.417459260048899;

double T_delta = 0.0025;

//sensor arrays
BLA::Matrix<3> y;
BLA::Matrix<3> Y;

//States
BLA::Matrix<7> states = {-1.613284840172769, 0.6630891035051645, 0.060518561228639606, -0.09560653458572338, 0.32300964003141536, 0.09697863711803018, -0.35906126562900564};
auto dynamicStates(states.Submatrix<4, 1>(0, 0));
auto biasStates(states.Submatrix<3, 1>(4, 0));

//for all large equations
class Eq {
    private:
        
        double Bx, By, Btheta;
        double theta, thetaD, x;
        double nps, npc, nps2, npc2;
        double standardD[3] = {0.5, 0.5, 0.34};
        double tau[3] = {200, 200, 250};
        long fact = 0;

        BLA::Matrix<3> sensors;
        BLA::Matrix<7, 7> expm;
        BLA::Matrix<7, 7, Eye<double>> diag;
        BLA::Matrix<7, 7> a_mul;

        BLA::Matrix<4, 4> A = {0, 1, 0, 0,
                               0, 0, M_beam*R_beam*R_wheel*g/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0,
                               0, 0, 0, 1,
                               0, 0, M_beam*R_beam*g/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0};
        
        BLA::Matrix<4> B = {0,
                               R_wheel*(I_beam + M_beam*pow(R_beam, 2) + M_beam*R_beam*R_wheel)/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)),
                               0,
                               (-I_wheel - M_beam*R_beam*R_wheel - M_beam*pow(R_wheel, 2) - M_wheel*pow(R_wheel, 2))/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2))};

        BLA::Matrix<4> K = {-10.0, -19.13473824, -207.22948091, -51.02194395};

    public:
        
        BLA::Matrix<4, 7> H = {0, 0, (I_beam*g + I_wheel*g + 2*M_beam*pow(R_beam, 2)*g + 3*M_beam*R_beam*R_wheel*g + M_beam*R_beam*g*R_acc + M_beam*pow(R_wheel, 2)*g + M_wheel*pow(R_wheel, 2)*g)/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0, 1, 0, 0,
                               0, 0, (M_beam*pow(R_beam, 2)*g + M_beam*R_beam*R_wheel*g)/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0, 0, 1, 0,
                               0, 0, 0, 1, 0, 0, 1};

        BLA::Matrix<7, 7> T = {0, 1, 0, 0, 0, 0, 0,
                               0, 0, M_beam*R_beam*R_wheel*g/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0, 0, 0, 0,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 0, M_beam*R_beam*g/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)), 0, 0, 0, 0,
                               0, 0, 0, 0, -1/tau[0], 0, 0,
                               0, 0, 0, 0, 0, -1/tau[1], 0,
                               0, 0, 0, 0, 0, 0, -1/tau[2]};

        BLA::Matrix<7, 7> q = {pow(0.00001, 2), 0, 0, 0, 0, 0, 0,
                               0, pow(0.00001, 2), 0, 0, 0, 0, 0,
                               0, 0, pow((0.0001 * 3.14159 / 180), 2), 0, 0, 0, 0,
                               0, 0, 0, pow((0.0005 * 3.14159 / 180), 2), 0, 0, 0,
                               0, 0, 0, 0, pow(2 * standardD[0], 2) / tau[0], 0, 0,
                               0, 0, 0, 0, 0, pow(2 * standardD[1], 2) / tau[1], 0,
                               0, 0, 0, 0, 0, 0, pow(2 * standardD[2], 2) / tau[2]};

        BLA::Matrix<3, 3> Y = {pow(0.003, 2), 0, 0,
                               0, pow(0.003, 2), 0,
                               0, 0, pow((0.05 * 3.14159 / 180), 2)};

        
        BLA::Matrix<7, 7> P = {0.01, 0, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0, 0,
                               0, 0, 0.01, 0, 0, 0, 0,
                               0, 0, 0, 0.01, 0, 0, 0,
                               0, 0, 0, 0, pow(standardD[0], 2), 0, 0,
                               0, 0, 0, 0, 0, pow(standardD[1], 2), 0,
                               0, 0, 0, 0, 0, 0, pow(standardD[2], 2)};
        
        void linSolve(BLA::Matrix<4, 1, BLA::Reference<BLA::Array<7, 1, float>>> dynamic, BLA::Matrix<3, 1, BLA::Reference<BLA::Array<7, 1, float>>> bias, double St_acc, double T_delta) {
            //linear solve of dynamics
            BLA::Matrix<4> statesD =  A * dynamic + B * St_acc;
            dynamic += statesD * T_delta;

            //propagating biases
            Bx = bias(0);
            By = bias(1);
            Btheta = bias(2);
            bias(0) += -Bx / tau[0] * T_delta;
            bias(1) += -By / tau[1] * T_delta;
            bias(2) += -Btheta / tau[2] * T_delta;
        };

        BLA::Matrix<3> sensorSolve(BLA::Matrix<4> dynamic, BLA::Matrix<3> bias, double St_acc) {
            //set equation values
            nps = sin(dynamic(2));
            npc = cos(dynamic(2));
            nps2 = sin(dynamic(2) * 2);
            npc2 = cos(dynamic(2) * 2);
            theta = dynamic(2);
            thetaD = dynamic(3);
            
            //make sensor guess(y)
            sensors = {(I_beam*R_wheel*St_acc*npc + I_beam*g*nps - I_wheel*R_acc*St_acc - I_wheel*R_beam*St_acc + I_wheel*g*nps - M_beam*R_acc*R_beam*R_wheel*St_acc*npc + M_beam*R_acc*R_beam*R_wheel*pow(thetaD, 2)*nps + M_beam*R_acc*R_beam*g*nps - M_beam*R_acc*pow(R_wheel, 2)*St_acc + M_beam*pow(R_beam, 2)*R_wheel*pow(thetaD, 2)*nps + 2*M_beam*pow(R_beam, 2)*g*nps + M_beam*R_beam*pow(R_wheel, 2)*St_acc*npc2/2 - M_beam*R_beam*pow(R_wheel, 2)*St_acc/2 + M_beam*R_beam*pow(R_wheel, 2)*pow(thetaD, 2)*nps2/2 + 3*M_beam*R_beam*R_wheel*g*nps2/2 + M_beam*pow(R_wheel, 2)*g*nps - M_wheel*R_acc*pow(R_wheel, 2)*St_acc - M_wheel*R_beam*pow(R_wheel, 2)*St_acc + M_wheel*pow(R_wheel, 2)*g*nps)/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)),(R_acc*pow(thetaD, 2)*(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)) - (R_beam*(pow(thetaD, 2)*(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2))*npc - (I_wheel*St_acc + M_beam*R_beam*R_wheel*St_acc*npc - M_beam*R_beam*R_wheel*pow(thetaD, 2)*nps - M_beam*R_beam*g*nps + M_beam*pow(R_wheel, 2)*St_acc + M_wheel*pow(R_wheel, 2)*St_acc)*nps) + g*(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)))*npc - (2*I_beam*R_beam*pow(thetaD, 2)*nps - 2*I_beam*R_wheel*St_acc + 2*I_wheel*R_beam*St_acc*npc + 2*I_wheel*R_beam*pow(thetaD, 2)*nps + 2*M_beam*pow(R_beam, 3)*pow(thetaD, 2)*nps + M_beam*pow(R_beam, 2)*R_wheel*St_acc*npc2 - M_beam*pow(R_beam, 2)*R_wheel*St_acc + M_beam*pow(R_beam, 2)*R_wheel*pow(thetaD, 2)*nps2 - M_beam*pow(R_beam, 2)*g*nps2 - 2*M_beam*R_beam*R_wheel*g*nps + 2*M_wheel*R_beam*pow(R_wheel, 2)*St_acc*npc + 2*M_wheel*R_beam*pow(R_wheel, 2)*pow(thetaD, 2)*nps)*npc/2)/(I_beam + I_wheel + M_beam*pow(R_beam, 2) + 2*M_beam*R_beam*R_wheel*npc + M_beam*pow(R_wheel, 2) + M_wheel*pow(R_wheel, 2)),thetaD};

            //add biases
            sensors += bias;
            
            return sensors;
        };

        double solveControl(BLA::Matrix<4> states_goal) {
            //operate controller
            St_acc = (-~K * states_goal)(0);
            return St_acc;
        };

        BLA::Matrix<7, 7> Expm(Matrix<7, 7> t, int order, float T_delta) {
            Matrix<7, 7> a = t * T_delta;
            expm = diag;
            a_mul = diag;
            fact = 1;

            for (int i = 1; i <= order; i++) {
              fact = fact * i;
              a_mul *= a;
              expm += a_mul / fact;
            }
            return expm;
        };
};

Eq Equation;

void setup() {
    Serial.begin(115200);
};

void loop() {};
