#ifndef DIALOG_CLIENT_H
#define DIALOG_CLIENT_H

#include <QDialog>
#include <QTcpServer>
#include "clientLocal.h"

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QTcpServer;
class QTcpSocket;
class QAction;
QT_END_NAMESPACE

class Dialog_Client : public QDialog
{
    Q_OBJECT

public:
    Dialog_Client(QWidget *parent = 0);
    void start();
    void inner_connection();

public slots:
	void loadFile();
	void runRawSeed();
	void runCarmon();
	void runSICK();
	void runFusion();
	void connectACK();
	void displayError(QAbstractSocket::SocketError socketError);
private:
    // QProgressBar *clientProgressBar;
    QLabel *clientStatusLabel;
    QLineEdit* editfileText;
    QString m_file_name;

	QString m_server_ip;
	unsigned int m_server_port;
	QLabel* m_label_server_ip;
	QLabel* m_label_server_port;
	QLineEdit* m_edit_server_ip;
	QLineEdit* m_edit_server_port;

	QLabel* m_label_sick1;
	QLabel* m_label_sick2;
	QLineEdit* m_edit_sick_ip1;
	QLineEdit* m_edit_sick_ip2;
	QLineEdit* m_edit_sick_port1;
	QLineEdit* m_edit_sick_port2;
	QString m_sick1_ip;
	QString m_sick2_ip;
	unsigned int m_sick1_port;
	unsigned int m_sick2_port;
	vector<string> v_sick_ips;
	vector<unsigned int> v_sick_ports;

    QPushButton *openfileBtn;
    QPushButton *RawSeedBtn;
    QPushButton *CarmonBtn;
    QPushButton *SICKBtn;
    QPushButton *FusionBtn;
    QPushButton *connectButton;
    QPushButton *quitButton;
    // QDialogButtonBox *buttonBox;

    CClientLocal tcpClient;
    QThread clientThread;
};

#endif
