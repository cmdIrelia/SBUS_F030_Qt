#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QUdpSocket>
#include <QTimer>
#include <QPlainTextEdit>
#include "joysitck_xbox360.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    Joysitck_Xbox360 *pJst;

    QTimer *timerRefreshUI;

    QSerialPort *serialPort1;

private:
    void SbusGenerator(Joysitck_Xbox360* jst);

    void LogWnd_WriteLine(QPlainTextEdit *logWnd, QString line);
    void LogWnd_WriteLine(QString line);
    QPlainTextEdit *logWnd;

    bool use_serialPortToCom;   //使用串口或者UDP发送数据

    int use_postionalOrIncremental;    //使用位置式或者增量式总距
    int realThrottle;

    QUdpSocket* m_sender;   //udp发送端

private slots:
    void refreshUI_timeout();

private:
    enum THROTTLE_MODE {
      THROTTLE_POSTIONAL,
      THROTTLE_INCREMENTAL
    };

private slots:

    void axisLeftXChanged(qint32 value);
    void axisLeftYChanged(qint32 value);
    void axisRightXChanged(qint32 value);
    void axisRightYChanged(qint32 value);

    void buttonAChanged(bool value);
    void buttonBChanged(bool value);
    void buttonXChanged(bool value);
    void buttonYChanged(bool value);

    void buttonUpChanged(bool value);
    void buttonDownChanged(bool value);
    void buttonLeftChanged(bool value);
    void buttonRightChanged(bool value);

    void buttonLeftBreak(bool value);
    void buttonRightBreak(bool value);

    void buttonLeftTrigger(bool value);
    void buttonRightTrigger(bool value);

    void connectedChanged(bool value);
//    void deviceIdChanged(int value);
//    void nameChanged(QString value);

    void on_pushButton_clicked();
    void on_radioButton_serial_toggled(bool checked);
    void on_radioButton_udp_toggled(bool checked);
    void on_pushButton_openSerialPort_clicked();
    void on_pushButton_openUdp_clicked();
    void on_checkBox_toggled(bool checked);
};
#endif // MAINWINDOW_H
