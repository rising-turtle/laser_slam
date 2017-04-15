
// NKJ 20130321
// length = 13
#include "JetFire.h"

JetFire::JetFire()
{
}
JetFire::~JetFire()
{
}
void JetFire::JetFireCmd_Pack(char Wheel, float Velocity, unsigned int mTime, unsigned char *Cmd_Ep)
{
    unsigned int speed;

// DuiLie    mTime = mTime / 5; // NKJ 20130322
    
    //  $L+0001T0005#
    Cmd_Ep[0] = '$';
    Cmd_Ep[1] = Wheel; // 'L' or 'R'
    if(Velocity >= 0)
    {
        Cmd_Ep[2] = '+';
        speed = (unsigned int)Velocity;
    }
    else
    {
        Cmd_Ep[2] = '-';
        speed = (unsigned int)(-Velocity);
    }
    Cmd_Ep[3] = speed / 1000;
    Cmd_Ep[4] = (speed - Cmd_Ep[3] * 1000) / 100;
    Cmd_Ep[5] = (speed - Cmd_Ep[3] * 1000 - Cmd_Ep[4] * 100) / 10;
    Cmd_Ep[6] = (speed - Cmd_Ep[3] * 1000 - Cmd_Ep[4] * 100 - Cmd_Ep[5]*10);
    Cmd_Ep[3] = Cmd_Ep[3] + '0';
    Cmd_Ep[4] = Cmd_Ep[4] + '0';
    Cmd_Ep[5] = Cmd_Ep[5] + '0';
    Cmd_Ep[6] = Cmd_Ep[6] + '0';
    Cmd_Ep[7] = 'T';
    
    Cmd_Ep[8] = mTime / 1000;
    Cmd_Ep[9] = (mTime - Cmd_Ep[8] * 1000) / 100;
    Cmd_Ep[10] = (mTime - Cmd_Ep[8] * 1000 - Cmd_Ep[9] * 100) / 10;
    Cmd_Ep[11] = (mTime - Cmd_Ep[8] * 1000 - Cmd_Ep[9] * 100 - Cmd_Ep[10]*10);
    Cmd_Ep[8] = Cmd_Ep[8] + '0';
    Cmd_Ep[9] = Cmd_Ep[9] + '0';
    Cmd_Ep[10] = Cmd_Ep[10] + '0';
    Cmd_Ep[11] = Cmd_Ep[11] + '0';
    Cmd_Ep[12] = '#';
}

// NKJ 20130321
void JetFire::Send_JetFireCmd(float Wheel_L, float Wheel_R, unsigned int mTime_L, unsigned int mTime_R,unsigned char* Cmd_Ep)
{
    //unsigned char Cmd_Ep[32];

/*	Cmd_Ep[0] ='@';
    Cmd_Ep[1] ='%';
	Cmd_Ep[2] ='N';
	JetFireCmd_Pack('L', Wheel_L, mTime_L, &Cmd_Ep[3]);
	Cmd_Ep[16] = 'N';
	JetFireCmd_Pack('R', Wheel_R, mTime_R, &Cmd_Ep[17]);
	Cmd_Ep[30] ='N';*/

	JetFireCmd_Pack('L', Wheel_L, mTime_L, &Cmd_Ep[0]);
    Cmd_Ep[13] = 'N';
    JetFireCmd_Pack('R', Wheel_R, mTime_R, &Cmd_Ep[14]);
 /*  Cmd_Ep[29] ='@';
    Cmd_Ep[30] ='%';
    Cmd_Ep[31] ='N';*/
}

void JetFire::Send_GetStatus(unsigned char *pcData,int nDataLen)
{
	pcData[0]='@';
	pcData[1]='%';
}

void JetFire::JetFireRot_Pack(float RotDegree, float Rot_Velocity, unsigned char *Cmd_Ep)
{
    unsigned int speed;

// DuiLie    mTime = mTime / 5; // NKJ 20130322

    //  $Z+0001T0050#
    Cmd_Ep[0] = '$';
    Cmd_Ep[1] = 'Z'; // 'Z' or 'z'
    if(RotDegree >= 0)
    {
        Cmd_Ep[2] = '+';
        speed = (unsigned int)RotDegree;
    }
    else
    {
        Cmd_Ep[2] = '-';
        speed = (unsigned int)(-RotDegree);
    }
    Cmd_Ep[3] = speed / 1000;
    Cmd_Ep[4] = (speed - Cmd_Ep[3] * 1000) / 100;
    Cmd_Ep[5] = (speed - Cmd_Ep[3] * 1000 - Cmd_Ep[4] * 100) / 10;
    Cmd_Ep[6] = (speed - Cmd_Ep[3] * 1000 - Cmd_Ep[4] * 100 - Cmd_Ep[5]*10);
    Cmd_Ep[3] = Cmd_Ep[3] + '0';
    Cmd_Ep[4] = Cmd_Ep[4] + '0';
    Cmd_Ep[5] = Cmd_Ep[5] + '0';
    Cmd_Ep[6] = Cmd_Ep[6] + '0';
    Cmd_Ep[7] = 'T';

    Cmd_Ep[8] = Rot_Velocity / 1000;
    Cmd_Ep[9] = (Rot_Velocity - Cmd_Ep[8] * 1000) / 100;
    Cmd_Ep[10] = (Rot_Velocity - Cmd_Ep[8] * 1000 - Cmd_Ep[9] * 100) / 10;
    Cmd_Ep[11] = (Rot_Velocity - Cmd_Ep[8] * 1000 - Cmd_Ep[9] * 100 - Cmd_Ep[10]*10);
    Cmd_Ep[8] = Cmd_Ep[8] + '0';
    Cmd_Ep[9] = Cmd_Ep[9] + '0';
    Cmd_Ep[10] = Cmd_Ep[10] + '0';
    Cmd_Ep[11] = Cmd_Ep[11] + '0';
    Cmd_Ep[12] = '#';
}
