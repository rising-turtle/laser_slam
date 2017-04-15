#ifndef TEST_DIAG_H
#define TEST_DIAG_H

#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <fstream>

using namespace std;

class CPolarMatch;
class CPoints;
struct _PMScan;

class TestDiag : public QWidget
{
	Q_OBJECT
public:
	TestDiag();
	~TestDiag();
Q_SIGNALS:
	void send2SLine(float,float,float,float);
	void sendMapInfo(float*,float*,int,double*,double*,double*);
	void paintReady();
public Q_SLOTS:
	void loadFile();
	void readFile();
	void showScan(int step = 10);
public:
	void send2Display(struct _PMScan&);
	QPushButton* m_openFile;
	QPushButton* m_runFile;
	QPushButton* m_nextScan;
	QPushButton* m_quit;
	CPolarMatch* m_pPSM;
	CPoints * m_pointDis;
	ifstream* m_inputFile;
};

#endif
