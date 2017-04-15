/*
 * CSICK.h
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#ifndef CSICK_H_
#define CSICK_H_
#include "CObs2DScan.h"
#include "CClientSocket.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <pthread.h>

#define APPERTURE           4.712385    // in radian <=> 270°

using namespace std;


class CSICK
{

public:
	/** Constructor.
	 * Note that there is default arguments, here you can customize IP Adress and TCP Port of your device.
	 */
	CSICK(std::string _ip=string("192.168.0.1"), unsigned int _port=2111);
	/** Destructor.
	 * Close communcation with the device, and free memory.
	 */
	virtual ~CSICK();
	/** This function acquire a laser scan from the device. If an error occured, hardwareError will be set to true.
	 * The new laser scan will be stored in the outObservation argument.
	 *
	 * \exception This method throw exception if the frame received from the LMS 100 contain the following bad parameters :
	 *  * Status is not OK
	 *  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).
	 */
	void doProcessSimple(bool &outThereIsObservation, CObs2DScan &outObservation, bool &hardwareError);

	//new for dual sick
	void doProcessSimple( );
	bool setSick_A(string, unsigned int);
	bool setSick_B(string, unsigned int);
	void runSick_A();
	void runSick_B();
	//static int getScan_A(float* , TTimeStamp& );
	static int getScan_A_SLAM(float* , TTimeStamp& );
	static int getScan_A_OD(float* , TTimeStamp& );
	//static int getScan_B(float* , TTimeStamp& );
	static int getScan_B_SLAM(float* , TTimeStamp& );
	static int getScan_B_OD(float* , TTimeStamp& );
	CObs2DScan m_scan_A;
	CObs2DScan m_scan_B;
	CClientSocket m_client_A;
	CClientSocket m_client_B;
	bool m_connected_A;
	bool m_connected_B;

	pthread_mutex_t mutex_read_A_SLAM;
	pthread_mutex_t mutex_read_A_OD;

	pthread_mutex_t mutex_read_B_SLAM;
	pthread_mutex_t mutex_read_B_OD;
	float* m_pScan_A_SLAM; // for SLAM
	float* m_pScan_A_OD; // for obstacle detection
	float* m_pScan_B_SLAM; // for SLAM
	float* m_pScan_B_OD; // for obstacle detection
	bool bFirst_A;
	bool bFirst_B;
	int scan_A_num;
	int scan_B_num;
	bool scan_A_Ready_SLAM;
	bool scan_A_Ready_OD;
	bool scan_B_Ready_SLAM;
	bool scan_B_Ready_OD;

	TTimeStamp scan_A_t_SLAM;
	TTimeStamp scan_A_t_OD;
	TTimeStamp scan_B_t_SLAM;
	TTimeStamp scan_B_t_OD;

	/** This method must be called before trying to get a laser scan.
	 */
	bool turnOn();
	/** This method could be called manually to stop communication with the device. Method is also called by destructor.
	 */
	bool turnOff();

	/** A method to set the sensor pose on the robot.
	 * Equivalent to setting the sensor pose via loading it from a config file.
	 */
	void setSensorPose(const CPose3D& _pose);

	void setSensorLabel(string );

	/** This method should be called periodically. Period depend on the process_rate in the configuration file.
	 */
	void  doProcess();

	/** Initialize the sensor according to the parameters previously read in the configuration file.
	 */
	void initialize();

	static CSICK * m_pCSICK;
private :
	string                  m_ip;
	unsigned int            m_port;
	CClientSocket        m_client;
	bool                    m_turnedOn;
	string					m_cmd;
	bool                    m_connected;
	unsigned int            m_scanFrequency;    // en hertz
	double                  m_angleResolution;  // en degrés
	double                  m_startAngle;       // degrés
	double                  m_stopAngle;        // degrés
	//CPose3D                 m_sensorPose;
	double                  m_maxRange;
	double                  m_beamApperture;
	std::string			m_sensorLabel;
	CObs2DScan m_scan;
	bool m_hdErr;
	bool m_obsErr;

	void generateCmd(const char *cmd);
	bool checkIsConnected();
	bool decodeScan(char *buf, CObs2DScan& outObservation);
	void sendCommand(const char *cmd);
	void roughPrint( char *msg );

	pthread_mutex_t mutex_read;


protected:
	/** Load sensor pose on the robot, or keep the default sensor pose.
	 */
	/*
	void  loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,
			const std::string	  &iniSection );
			*/

};


#endif /* CSICK_H_ */
