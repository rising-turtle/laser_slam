#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QTcpServer>
#include <QTcpSocket>
#include "serverGlobal.h"
#include "clientLocal.h"

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QTcpServer;
class QTcpSocket;
class QAction;
QT_END_NAMESPACE

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(QWidget *parent = 0);

public slots:
    void start();
    void updateServerProgress(qint64);
    void updateClientProgress(qint64);
    void displayError(QAbstractSocket::SocketError socketError);

private:
    QProgressBar *clientProgressBar;
    QProgressBar *serverProgressBar;
    QLabel *clientStatusLabel;
    QLabel *serverStatusLabel;

    QPushButton *startButton;
    QPushButton *quitButton;
    QDialogButtonBox *buttonBox;

    // QTcpServer tcpServer;
    CServerGlobal tcpServer;
    // QTcpSocket tcpClient;
    CClientLocal tcpClient;
    int bytesToWrite;
    int bytesWritten;
    int bytesReceived;
    int TotalBytes;
};

#endif
