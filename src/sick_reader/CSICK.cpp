/*
 * CSICK.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#include "CSICK.h"
#define _SICK_CONNECTION_TIME_OUT 1000 // ms

CSICK * CSICK::m_pCSICK;

CSICK::CSICK(string _ip, unsigned int _port):
m_ip(_ip),
m_port(_port),
m_client(),
m_turnedOn(false),
m_cmd(),
m_connected(false),
//m_sensorPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
m_maxRange(50.0),
m_beamApperture(0.5*M_PI/180.0),
m_sensorLabel("LMS151"),
bFirst_A(true),
bFirst_B(true),
scan_A_num(0),
scan_B_num(0),
scan_A_Ready_SLAM(false),
scan_A_Ready_OD(false),
scan_B_Ready_SLAM(false),
scan_B_Ready_OD(false)
{

	m_connected_A = false;
	m_connected_B = false;

	 pthread_mutex_init(&mutex_read_A_OD, NULL);
	 pthread_mutex_init(&mutex_read_A_SLAM, NULL);
	 pthread_mutex_init(&mutex_read_B_OD, NULL);
	 pthread_mutex_init(&mutex_read_B_SLAM, NULL);

	 m_pCSICK=this;
}

CSICK::~CSICK()
{
	if(m_connected)
		m_client.close();

	pthread_mutex_destroy(&mutex_read_A_OD);
	pthread_mutex_destroy(&mutex_read_A_SLAM);
	pthread_mutex_destroy(&mutex_read_B_OD);
	pthread_mutex_destroy(&mutex_read_B_SLAM);
	//    delete m_client;
	//    delete m_sensorPose;
}

void CSICK::setSensorLabel(string name)
{
	m_sensorLabel = name;
}

bool CSICK::checkIsConnected(void)
{
	if(m_connected) return true;
	else
	{
		try{
			m_client.connect(m_ip, m_port,_SICK_CONNECTION_TIME_OUT);
		}catch(std::exception &e)
		{
			printf(e.what());
			printf("FAILED TO CONNECT SICK \n");
			return false;
		}
	}
	m_connected = true;
	return true;
}
bool CSICK::turnOff()
{

	if(m_connected)
	{
		m_client.close();
		m_connected = false;
		pthread_mutex_destroy(&mutex_read_A_OD);
		pthread_mutex_destroy(&mutex_read_A_SLAM);
		pthread_mutex_destroy(&mutex_read_B_OD);
		pthread_mutex_destroy(&mutex_read_B_SLAM);
	}

	return true;

}


bool CSICK::turnOn()
{
	/** From the LMS100 datasheet : :
	 * * Login sMN SetAccessMode 03 F4724744
	 * * Set Scanarea and Resolution
	 * * sMN mLMPsetscancfg
	 * * SWN LMDscandatacfg 01 00 0 1 0 00 00 0 0 0 0 1
	 * * SMN mEEwriteall   Je ne le fais pas, car ca écrit en mémoire non volatile...
	 * * Request scan : sRN LMDscandata OR sEN LMDscandata
	 */
	size_t read;
	if(checkIsConnected())
	{

		try{
			// Dec format must with "-+"; Hex without "-+";
			{
				char msg[] = {"sMN SetAccessMode 03 F4724744"}; //Log in: authorized client
				char msgIn[100];

				sendCommand(msg);

				read = m_client.readAsync(msgIn, 100, 100, 100);  //18
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
			}
			/*
			 * it takes too long to reboot after using this command
			 * dont use it if u have to change res.
			 * if use it, please sleep for about 20seconds after the turn on in your main function
			 * */
			/*
			{
				char msg[] = {"sMN mLMPsetscancfg +5000 +1 +5000 -450000 +2250000"}; //set frequency and resolution (can not change angle range here)
				//char msg[] = {"sMN mLMPsetscancfg +5000 +1 +5000 0 0"};//for lms5xx
				char msgIn[100];
				sendCommand(msg);
				m_client.readAsync(msgIn, 100, 100, 100);
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
			}
			*/
			{
				char msg[] = {"sWN LMPoutputRange 1 +5000 -450000 +2250000"}; //configure measurement angle of the scan data for output (can not change res here)
				char msgIn[100];
				sendCommand(msg);
				m_client.readAsync(msgIn, 100, 100, 100);
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
			}
			{
				char msg[] = {"sWN LMDscandatacfg 01 00 0 1 0 00 00 0 0 0 0 +1"}; //configure scan data content
				//char msg[] = {"sWN LMDscandatacfg 0 0 0 0 0 0 0 0 0 0 0 +1"}; //for lms 5xx
				char msgIn[100];
				sendCommand(msg);
				m_client.readAsync(msgIn, 100, 100, 100);
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
			}
			{
				char msg[] = {"sMN mEEwriteall"}; //Store Parameters
				char msgIn[100];
				sendCommand(msg);
				m_client.readAsync(msgIn, 100, 100, 100);
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
				sleep(3); //need few seconds for coming messages
			}
			{
				char msg[] = {"sMN Run"}; //Log out: Return to the measurement mode after the configuration
				char msgIn[100];
				sendCommand(msg);
				m_client.readAsync(msgIn, 100, 100, 100);
				printf("message : %s\n",string(msgIn).c_str());
				if(!read) return false;
			}
			m_turnedOn = true;
		}catch(std::exception &e)
		{
			printf(e.what());
			return false;
		}
	}else
	{
		return false;
	}
	return true;
}


void CSICK::sendCommand(const char *cmd)
{
	generateCmd(cmd);
	if (!m_cmd.empty()) // one never knows...
		m_client.writeAsync(&m_cmd[0], m_cmd.size());
}

/** Add the start and end character.
 */
void CSICK::generateCmd(const char *cmd)
{
	if(strlen(cmd) > 995)
	{
		printf("la commande est trop longue\n");
		return;
	}
	m_cmd = format("%c%s%c",0x02,cmd,0x03);
}


bool CSICK::decodeScan(char* buff, CObs2DScan& outObservation)
{
	char *next;
	unsigned int idx = 0;
	unsigned int scanCount=0;
	char* ptr;
	//double factor;

	next = strtok_r(buff, " ", &ptr);


	while(next && scanCount==0)
	{
		//cout <<m_sensorLabel<<" Interpreting : " << next << endl;
		switch(++idx)
		{
		case 1:
			if(strncmp(&next[1], "sRA", 3) && strncmp(&next[1], "sSN", 3)) return false;
			break;
		case 2 :
			if(strcmp(next, "LMDscandata")) return false;
			break;
		case 6 :
			if(strcmp(next, "0"))
			{
				printf("STATUS error on LMS100");
				//THROW_EXCEPTION("STATUS error on LMS100");
				return false;
			}
			//printf("STATUS Ok.\n");
			break;
		case 21 :
			if(strcmp(next, "DIST1"))
			{
				printf("LMS SICK is not configured to send ditances.");
				//THROW_EXCEPTION("LMS100 is not configured to send ditances.");
				return false;
			}
			//printf("Distance : OK\n");
			break;
		case 22 :
			//factor = strtod(next, NULL);
			break;
		case 26 :
			scanCount = strtoul(next, NULL, 16);
			//printf("Scan Count : %d\n", scanCount);
			break;
		default :
			break;
		}
		next = strtok_r(NULL, " ", &ptr);
	}
	outObservation.aperture = APPERTURE;
	outObservation.rightToLeft = false;
	outObservation.stdError = 0.012f;
	//outObservation.sensorPose = m_sensorPose;
	outObservation.beamAperture = m_beamApperture;
	outObservation.maxRange = m_maxRange;
	outObservation.timestamp				= getCurrentTime();
	outObservation.sensorLabel             = m_sensorLabel;

	outObservation.scan.clear();
	outObservation.validRange.clear();
	unsigned int i;
	for(i = 0 ; i < scanCount && next; i++, next = strtok_r(NULL, " ", &ptr))
	{
		outObservation.scan.push_back(double(strtoul(next, NULL, 16))/1000.0);
		outObservation.validRange.push_back(outObservation.scan[i] <= outObservation.maxRange);
	}
	return i>=scanCount;
}

void CSICK::doProcessSimple(bool &outThereIsObservation, CObs2DScan &outObservation, bool &hardwareError)
{
	if(!m_turnedOn)
	{
		hardwareError = true;
		outThereIsObservation = false;
		return;
	}
	hardwareError = false;

	char msg[] = {"sRN LMDscandata"};
	sendCommand(msg);
	char buffIn[16*1024];
	//size_t read = m_client.readAsync(buffIn, sizeof(buffIn), 100, 100);
	//cout << "read :" << read << endl;
	//while(m_client.readAsync(buffIn, sizeof(buffIn), 100, 100)) cout << "Lit dans le vent" << endl;
	m_client.readAsync(buffIn, sizeof(buffIn), 20, 20); // 20ms is the time to wait the data depending on the freq

	if(decodeScan(buffIn, outObservation))
	{
		// Do filter:
		//this->filterByExclusionAreas( outObservation );
		//this->filterByExclusionAngles( outObservation );
		outThereIsObservation = true;
		hardwareError = false;
	}else
	{
		hardwareError = true;
		outThereIsObservation = false;
		//printf("doProcessSimple failed\n");
	}
}



void CSICK::doProcessSimple( )
{

	if(!m_turnedOn)
	{
		m_hdErr = true;
		m_obsErr = false;
		return;
	}
	m_hdErr = false;

	while(m_connected)
	{
		char msg[] = {"sRN LMDscandata"};
		sendCommand(msg);
		char buffIn[16*1024];
		//size_t read = m_client.readAsync(buffIn, sizeof(buffIn), 100, 100);
		//cout << "read :" << read << endl;
		//while(m_client.readAsync(buffIn, sizeof(buffIn), 100, 100)) cout << "Lit dans le vent" << endl;
		m_client.readAsync(buffIn, sizeof(buffIn), 20, 20); // 20ms is the time to wait the data depending on the freq

		{
			pthread_mutex_lock (&mutex_read);
			if(decodeScan(buffIn, m_scan))
			{
				// Do filter:
				//this->filterByExclusionAreas( outObservation );
				//this->filterByExclusionAngles( outObservation );
				m_obsErr = true;
				m_hdErr = false;
			}else
			{
				m_hdErr = true;
				m_obsErr = false;
				//printf("doProcessSimple failed\n");
			}
			pthread_mutex_unlock (&mutex_read);
		}
	}
}


bool CSICK::setSick_A(string ip, unsigned int port)
{
	try{
	m_client_A.connect(ip, port,_SICK_CONNECTION_TIME_OUT);
	}catch(std::exception &e)
	{
		printf(e.what());
		printf("FAILED TO CONNECT SICKA \n");
		return false;
	}
	m_connected_A = true;

	char msg[] = {"sMN Run"}; //Log out: Return to the measurement mode after the configuration
	char msgIn[100];

	string cmd;
	cmd = format("%c%s%c",0x02,msg,0x03);

	m_client_A.writeAsync(&cmd[0], cmd.size());
	m_client_A.readAsync(msgIn, 100, 100, 100);

	printf("setSick_A message : %s\n",string(msgIn).c_str());


}

bool CSICK::setSick_B(string ip, unsigned int port)
{
	try{
		m_client_B.connect(ip, port,_SICK_CONNECTION_TIME_OUT);
	}catch(std::exception &e)
	{
		printf(e.what());
		printf("FAILED TO CONNECT SICKB \n");
		return false;
	}
	m_connected_B = true;

	char msg[] = {"sMN Run"}; //Log out: Return to the measurement mode after the configuration
	char msgIn[100];

	string cmd;
	cmd = format("%c%s%c",0x02,msg,0x03);

	m_client_B.writeAsync(&cmd[0], cmd.size());
	m_client_B.readAsync(msgIn, 100, 100, 100);

	printf("setSick_B message : %s\n",string(msgIn).c_str());

}


void CSICK::runSick_A( )
{

	char buffIn[16*1024];
	string cmd;
	while(m_connected_A)
	{

		char msg[] = {"sRN LMDscandata"};
		cmd = format("%c%s%c",0x02,msg,0x03);
		m_client_A.writeAsync(&cmd[0], cmd.size());
		m_client_A.readAsync(buffIn, sizeof(buffIn), 20, 20); // 20ms is the time to wait the data depending on the freq

		{


			decodeScan(buffIn, m_scan_A);

			if(m_scan_A.scan.size()>0)
			{
				if(bFirst_A)
				{
					scan_A_num = m_scan_A.scan.size();
					m_pScan_A_SLAM = new float[scan_A_num];
					m_pScan_A_OD = new float[scan_A_num];
					bFirst_A = false;
				}

				// copy for slam
				{
					pthread_mutex_lock (&mutex_read_A_SLAM);
					memcpy(m_pScan_A_SLAM, &(m_scan_A.scan[0]),scan_A_num*sizeof(m_scan_A.scan[0]));
					scan_A_t_SLAM = m_scan_A.timestamp;
					scan_A_Ready_SLAM = true;
					pthread_mutex_unlock (&mutex_read_A_SLAM);
				}
				//copy for obstacle detection
				{

					pthread_mutex_lock (&mutex_read_A_OD);
					memcpy(m_pScan_A_OD, &(m_scan_A.scan[0]),scan_A_num*sizeof(m_scan_A.scan[0]));
					scan_A_t_OD = m_scan_A.timestamp;
					scan_A_Ready_OD = true;
					pthread_mutex_unlock (&mutex_read_A_OD);
				}

				//cout<<"m_scan_A: "<<m_scan_A.scan.size()<<endl;
			}

		}
	}
}

void CSICK::runSick_B( )
{
	char buffIn[16*1024];
	string cmd;
	while(m_connected_B)
	{
		char msg[] = {"sRN LMDscandata"};
		cmd = format("%c%s%c",0x02,msg,0x03);
		m_client_B.writeAsync(&cmd[0], cmd.size());
		m_client_B.readAsync(buffIn, sizeof(buffIn), 20, 20); // 20ms is the time to wait the data depending on the freq

		{
			decodeScan(buffIn, m_scan_B);
			if(m_scan_B.scan.size()>0)
			{
				if(bFirst_B)
				{
					bFirst_B = false;
					scan_B_num = m_scan_B.scan.size();
					m_pScan_B_SLAM = new float[scan_B_num];
					m_pScan_B_OD = new float[scan_B_num];
				}

				// copy for slam
				{
					pthread_mutex_lock (&mutex_read_B_SLAM);
					memcpy(m_pScan_B_SLAM, &(m_scan_B.scan[0]),scan_B_num*sizeof(m_scan_B.scan[0]));
					scan_B_t_SLAM = m_scan_B.timestamp;
					scan_B_Ready_SLAM = true;
					pthread_mutex_unlock (&mutex_read_B_SLAM);
				}
				//copy for obstacle detection
				{
					pthread_mutex_lock (&mutex_read_B_OD);
					memcpy(m_pScan_B_OD, &(m_scan_B.scan[0]),scan_B_num*sizeof(m_scan_B.scan[0]));
					scan_B_t_OD = m_scan_B.timestamp;
					scan_B_Ready_OD = true;
					pthread_mutex_unlock (&mutex_read_B_OD);
				}

				cout<<"m_scan_B: "<<m_scan_B.scan.size()<<endl;
			}
			//cout<<"m_scan_B: "<<m_scan_B.scan.size()<<endl;

		}
	}

}


int CSICK::getScan_A_SLAM(float* scan, TTimeStamp& t)
{
	int nRtn=0;
	pthread_mutex_lock (&m_pCSICK->mutex_read_A_SLAM);
	if(m_pCSICK->scan_A_Ready_SLAM)
	{
		memcpy(scan, m_pCSICK->m_pScan_A_SLAM, m_pCSICK->scan_A_num*sizeof(float));
		t = m_pCSICK->scan_A_t_SLAM;
		nRtn= 1;
		m_pCSICK->scan_A_Ready_SLAM = false;
	}
	else
	{
		nRtn= 0;
	}
	pthread_mutex_unlock (&m_pCSICK->mutex_read_A_SLAM);
	return nRtn;

}

int CSICK::getScan_A_OD(float* scan, TTimeStamp& t)
{
	int nRtn=0;

	pthread_mutex_lock (&m_pCSICK->mutex_read_A_OD);

	if(m_pCSICK->scan_A_Ready_OD)
	{
		memcpy(scan, m_pCSICK->m_pScan_A_OD, m_pCSICK->scan_A_num*sizeof(float));
		t = m_pCSICK->scan_A_t_OD;
		nRtn= 1;
		m_pCSICK->scan_A_Ready_OD = false;
	}
	else
	{
		nRtn= 0;
	}

	pthread_mutex_unlock (&m_pCSICK->mutex_read_A_OD);
	return nRtn;

}

int CSICK::getScan_B_SLAM(float* scan, TTimeStamp& t)
{
	int nRtn=0;

	pthread_mutex_lock (&m_pCSICK->mutex_read_B_SLAM);

	if(m_pCSICK->scan_B_Ready_SLAM)
	{
		memcpy(scan, m_pCSICK->m_pScan_B_SLAM, m_pCSICK->scan_B_num*sizeof(float));
		t = m_pCSICK->scan_B_t_SLAM;
		nRtn= 1;
		m_pCSICK->scan_B_Ready_SLAM = false;
	}
	else
	{
		nRtn= 0;
	}

	pthread_mutex_unlock (&m_pCSICK->mutex_read_B_SLAM);
	return nRtn;

}

int CSICK::getScan_B_OD(float* scan, TTimeStamp& t)
{
	int nRtn=0;

	pthread_mutex_lock (&m_pCSICK->mutex_read_B_OD);

	if(m_pCSICK->scan_B_Ready_OD)
	{
		memcpy(scan, m_pCSICK->m_pScan_B_OD, m_pCSICK->scan_B_num*sizeof(float));
		t = m_pCSICK->scan_B_t_OD;
		m_pCSICK->scan_B_Ready_OD = false;
		nRtn= 1;
	}
	else
	{
		nRtn= 0;
	}

	pthread_mutex_unlock (&m_pCSICK->mutex_read_B_OD);
	return nRtn;

}
