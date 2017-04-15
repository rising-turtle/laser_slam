#ifndef __JetFire_H 
#define __JetFire_H

// NKJ 20130321
// length = 13

class JetFire
{
public:
	JetFire();
	~JetFire();
	static void JetFireCmd_Pack(char Wheel, float Velocity, unsigned int mTime, unsigned char *Cmd_Ep);
	static void Send_JetFireCmd(float Wheel_L, float Wheel_R, unsigned int mTime_L, unsigned int mTime_R,unsigned char* Cmd_Ep);
	static void Send_GetStatus(unsigned char *pcData,int nDataLen);

	static void JetFireRot_Pack(float RotDegree, float Rot_Velocity, unsigned char *Cmd_Ep);
};




#endif





